# Hard Fault Investigation — NXP FMU-V6XRT

**Status: SOAK IN PROGRESS, ON TRACK — 2026-08-29** (see "Soak test evidence" below —
this campaign has walked back three prior "fixed" calls that were declared on a quiet
window shorter than the required bar, so this doc deliberately does not say RESOLVED
until an instrument-counted run clears it)

## Summary

Starting around 2026-08-14, PXLABS Differential Rover units built on NXP FMU-V6XRT
(MIMXRT1176) hard-faulted intermittently in flight and on the bench — typically every
5–15 minutes, occasionally with clean windows of several hours. Root cause is an
**uncalibrated FlexSPI DLL read strobe** for XIP (execute-in-place) flash reads at
200MHz octal DDR: without per-boot calibration, the read strobe sampling point can land
close to the edge of the valid data window instead of its center, producing occasional
corrupted instruction fetches under load/thermal/voltage variation. The fix is
cherry-picking upstream PX4 **PR [#28141](https://github.com/PX4/PX4-Autopilot/pull/28141)**
("feat(fmu-v6xrt): calibrate FlexSPI DLL read strobe at boot"), authored by NXP engineer
Peter van der Perk. A continuous instrument-counted soak test with the fix applied is
in progress and on track to clear the 8h pass bar (see below for the exact numbers) —
treat this as strong, not yet final, evidence.

## Symptom

- Random hard faults, roughly every 5–15 min (historical median 5.4 min, p90 73 min),
  with the longest clean window ever observed pre-fix at 7.3h.
- Faulting task varied with system load (`uxrce_dds_client`, `wq:INS0`, `wq:uavcan`,
  etc.) — not tied to one subsystem or driver.
- Reproduced on 4 separate flight controllers across 2 sites ~5km apart.
- Reproduced on a board powered by USB only, off the vehicle, in a different building,
  at a statistically identical rate to on-vehicle operation.
- Reproduced on stock upstream PX4 v1.17.0 with no PXLABS changes — not caused by any
  PXLABS modification.

## What we ruled out

Each of the following was tested directly, not just reasoned about:

| Hypothesis | Result |
|---|---|
| One bad board | Reproduces on 4 separate FCs |
| Vehicle wiring / power / EMI / companion computer / CAN bus | Board off-vehicle on USB-only power in a different building faulted at the same rate |
| PXLABS code changes | Reproduces on stock upstream PX4 v1.17.0 |
| Corrupted stored firmware image | Flash bytes verified byte-for-byte against the ELF at the faulting PC, twice |
| FPU not enabled (classic NOCP cause) | `CPACR` read live at the fault — FPU genuinely enabled (`CP10=3 CP11=3`) |
| SD card / bad parameters | Reformatted and diffed/restored — fault persisted |
| Known RT1176 Rev 1.3 silicon errata (ERR011573 / ERR006940 / ERR011377 / ERR050396) | Desk-checked against all four — none match the observed signature |
| RAM vector table relocation into `.itcmfunc` (a real difference vs v1.16.2 found during this investigation) | **Refuted directly by hardware data** — `HFSR.VECTTBL=0` on every live catch and archived log, which rules out a bus fault during vector-table fetch. Also still present unfixed in v1.18.0-alpha1, so not version-specific either way. Do not re-raise this lead. |

## Live-caught fault signature (JTAG/SWD, 2026-08-27)

Three live catches via NXP MCU-Link probe + pyOCD:

- `CFSR` = `UNDEFINSTR`, `HFSR = 0x40000000` (FORCED only — `VECTTBL` bit **not** set)
- Faulting instruction's stored flash bytes exactly match the ELF at that address —
  i.e. the CPU executed something different from what is actually stored in flash,
  transiently, with no trace afterward.
- Faulting PCs consistently in the FlexSPI XIP window (`0x30xxxxxx`); matched 45 of 62
  archived post-mortem logs from before the probe was attached.
- All observed `NOCP` faults were on genuine FPU instructions — consistent with a bit
  flip landing in a VFP instruction's coprocessor field during fetch.

This pointed at a **marginal instruction fetch from XIP flash**, not software, not the
stored image, and not the vector table.

## Root cause

The NXP FMU-V6XRT boots and executes directly from FlexSPI NOR flash in **octal DDR
mode at 200MHz** — one of the most timing-critical XIP configurations the MIMXRT1176
supports, with no ECC on that memory region. The FlexSPI **DLL (delay-locked loop) read
strobe**, which times when the controller samples incoming data, was left at its
power-on-reset default rather than being calibrated per-boot. A read strobe that is
merely "locked" (per NXP spec) is not the same as being **centered** in the valid data
eye — under load, thermal drift, or voltage transients, a strobe sitting near the edge
of that window can occasionally sample a bit early or late, corrupting the fetched
instruction word. This matches every element of the observed signature: transient
(no persistent flash corruption), load-dependent (varies which task/timing wins the
race), confined to the XIP window, and undetectable by static byte-for-byte flash
verification (the flash contents are correct — only the *read* was wrong).

> **`LOCKED ≠ CENTRED` — this trap cost two days on this investigation, don't repeat it.**
> The FlexSPI DLL register (`FLEXSPI1 @ 0x400CC000`) was read early on: `DLLCR =
> 0x00400079` (DLLEN=1, **OVRDEN=0** — override off, ROM default in force) and `STS2 =
> 0x00000b33` → `ASLVSEL=12, AREFSEL=11`, with both lock bits set. Seeing the DLL
> "locked" per NXP's own spec, the DLL was struck off the suspect list for two days.
> The lock bits only say the delay-locked loop achieved lock — they say nothing about
> *where in the valid read window* it landed. A DLL can be perfectly locked at the very
> edge of the eye and still be marginal. Anyone re-treading this investigation who reads
> "DLL locked" and moves on will make the exact same mistake.

## The fix

Upstream PR **[PX4/PX4-Autopilot#28141](https://github.com/PX4/PX4-Autopilot/pull/28141)**
("feat(fmu-v6xrt): calibrate FlexSPI DLL read strobe at boot", merged to PX4 `main`
2026-08-04/05, commit `9f4bc80006caf7d5d8bcf809b4ccd046fb5eded6`), authored by NXP
engineer Peter van der Perk, adds an active DLL calibration routine at boot
(`board_app_initialize()` in `boards/px4/fmu-v6xrt/src/init.c`) that sweeps and centers
the read strobe in the valid window before the system starts relying on XIP execution,
instead of trusting the ROM's default "locked" placement. It logs the calibrated
midpoint (`g_dll_cal`) on first boot for diagnostic visibility.

This PR was found via the public forum post PXLABS opened asking for help
(2026-08-28) — van der Perk replied pointing at his own fix, and at a closed upstream
issue (#27735) describing an unrelated user's fmu-v6xrt rover hitting the identical
signature (`UNDEFINSTR`, garbage PC, load-dependent) that #28141 also resolved.

### What was done

1. Cherry-picked commit `9f4bc80006c` cleanly onto `pxlabs-v1.17.0-hardfault-debug`
   as `860013bab7` — zero conflicts, one file changed
   (`boards/px4/fmu-v6xrt/src/init.c`, +217/−13). **Verify a build actually has this
   fix by its git hash — `flight_sw_version` reads `1.17.0` identically with or
   without the cherry-pick and cannot be used to distinguish a fixed build.**
2. Built clean: flash 61.80% → 62.01% (+8664 B), SRAM 5.39% → 5.50% (+2112 B for
   `g_dll_cal`, symbol address `0x20252774` in this build), ITCM unchanged at 85.42%.
3. Flashed to hardware and soak-tested starting the evening of 2026-08-28.
4. Merged onto `pxlabs-v1.17.0-dev` as a fast-forward (no conflicts) and released as
   `v1.17.0-2.1.0`.

### Soak test evidence (instrument-counted, not just operator-observed)

Continuous measurement via `~/ros2_ws/tools/fc_soak.py` on the companion's board:

- Single boot at 2026-08-29 01:11:01, **zero reboots since**
- As of 2026-08-29 08:55 IST: **464.5 min (7.74h) uptime, 0 new fault logs**, SD-card
  fault baseline still 0 since soak start, real load running throughout (camera,
  `/scan`, `/scan_3d`, wheel odometry, uXRCE-DDS all up), `throttled=0x0`, 51°C
- The 8h pass bar (480 min) is reached at ~09:11 IST on this boot — **on track, but not
  yet crossed as of the last confirmed reading.** This bar exists because the fault
  era's own measured statistics put P(quiet ≥8h | bug still present) at 0% — so
  clearing it is treated as confirmation, not coincidence — but three earlier fixes in
  this campaign (charger, USB bench isolation, SD reformat) were each declared on a
  quiet window shorter than this bar and later withdrawn. Update this section (and the
  Status line at the top) once the 8h mark is confirmed crossed by instrument, not by
  elapsed wall-clock time alone.

## Known gap: not yet in any v1.18 upstream tag

PR#28141 landed on PX4 `main` on 2026-08-05. As of 2026-08-29, none of the upstream
v1.18 tags cut so far include it:

| Tag | Date | Contains PR#28141? |
|---|---|---|
| `v1.18.0-alpha1` | 2026-05-13 | No |
| `v1.18.0-beta1` | 2026-07-08 | No |
| `v1.18.0-beta2` | 2026-07-31 | No |

**If this project ever moves to a v1.18-based branch, PR#28141 must be cherry-picked
onto it again** — the same way it was here — or that board will reproduce this exact
hard fault. Re-check with `git merge-base --is-ancestor 9f4bc80006caf7d5d8bcf809b4ccd046fb5eded6 <tag>`
against whatever v1.18 tag is current at that time, since a later rc/GA tag may include
it by then.

## Separate bug noticed along the way (not part of this fix)

A distinct, unrelated fault class was also observed during this investigation: genuine
null-pointer dereferences inside uXRCE-DDS serializers. 15 `DACCVIOL` faults in the
corpus, 7 with `MMFAR` at small offsets from NULL (`0x00, 0x04, 0x08, 0x0a, 0x0c, 0x14,
0x20`). Confirmed at the instruction level (not just by symbol): 4 in
`ucdr_serialize_esc_status` at `str rX,[r4,#8]` / `bl __memcpy_veneer`; others in
`ucdr_serialize_vehicle_local_position`, `ucdr_serialize_vehicle_odometry`, and
`ucdr_serialize_input_rc`. This is independently fixable and worth reporting to the
uXRCE-DDS maintainers separately — **the FlexSPI DLL fix is not expected to remove
these and this document should not be read as implying it does.** Next step: watch
whether any of these still fire on the fixed build during the ongoing soak; a good
candidate for its own upstream issue regardless of outcome.

## Credits

- NXP engineer **Peter van der Perk** (`PetervdPerk-NXP`) — identified and fixed the
  root cause upstream, and pointed PXLABS at it via the forum thread.
- Live JTAG/SWD fault capture, fault-rate statistics, DLL register readings, and the
  ongoing instrument-counted soak test were produced by a companion debugging session
  with direct hardware probe access. Full raw record:
  `fc_hardfault_handoff_20260829.md` on the `Companion_Computer_Pxlabs` repo (`master`).
