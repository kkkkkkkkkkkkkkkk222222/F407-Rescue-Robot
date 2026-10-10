# Injury half-claw backoff and second audit

Upper baseline: `codex/gamepad-teleop@7a3673f`.

The later audit/motor updates are described in `UPPER_INJURY_AND_AUDIT_REQUEST.md`.
Use the cumulative `rdk_patches/0007-rescue-audit-and-mg310.patch` for a fresh
upper deployment; it includes 0004/0005/0006 and must not be reapplied blindly.

## Formal injury pickup

1. Keep the existing pre-grab and mode23 post-grab three-frame gates.
2. A legal single-injury post-grab result enters mode49, not mode22.
3. Move the left claw from 76 to 81 degrees and the right from 104 to 99
   degrees. Wait 600 ms for the half-claw posture.
4. Reverse at a configured 150 mm/s while holding the starting heading for
   2000 ms of commanded motion, including acceleration. This is a timed
   motion, not a guaranteed 300 mm displacement. PAUSE, command loss and
   invalid pose stop the motor; paused time is excluded.
5. Stop, lower the camera from 120 to 140 degrees, wait 500 ms, discard all
   earlier evidence and return to mode23. Claws remain half closed.
6. Require three new distinct audit IDs under the unchanged legality rules.
   Empty and illegal cargo use the existing open/release/separation paths.
7. A legal second audit enters mode50. Restore 76/104 degrees, preserve
   the accepted audit, wait the existing 2000 ms claw-close window, then
   report mode22 plus GRIPPER_CLOSED. Only then may NAV start.

During mode49, CLAW_VISIBLE, GRIPPER_CLOSED, AUDIT_READY and AUDIT_VALID
are false. Audit commands are acknowledged only as heartbeats. During
mode50, keep READY/VALID but report GRIPPER_CLOSED=false until completion.
Duplicate GRAB/audit commands cannot restart either physical action.

Ordinary/core pickups and INITIAL_STASH keep their existing flows. The
safe-zone sweep retrieval retains mode23->40 at its recorded staging
point: it recovers cargo that already passed the normal pickup gate, and
must not repeat the initial backoff away from that point. Mode48 staging
audit, single-cargo limits and delivery timeout behavior are unchanged.

## Upper integration

Apply `rdk_patches/0004-injury-half-claw-backoff.patch` at the upper repository
root, based on the branch/commit above. Update the upper program and F407
firmware together. Mode49 clears the pending first audit and publishes
non-stable empty heartbeat audits. Returning to mode23 resets the visual
frame floor again, so images captured during movement cannot count.
Mode50 preserves the pending second audit and waits for a fresh mode22
with closed claws before latching the actual cargo and destination.

## Verification

The rescue firmware builds with:

```powershell
cmake -S . -B build/InjuryVerify -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=Debug' '-DAPP_ACTIVE_MODE_OVERRIDE=APP_MODE_RESCUE_TASK' '-DAPP_AUTO_BACKUP=OFF'
cmake --build build/InjuryVerify --target WWW
gcc -std=c11 -O1 -ffunction-sections -fdata-sections -Itools/test_support -IMain/Inc tools/test_injury_backoff.c Main/Src/pid.c '-Wl,--gc-sections' -lm -o build/InjuryVerify/test_injury_backoff.exe
./build/InjuryVerify/test_injury_backoff.exe
```

The host harness executes the production Task/mechanism code with mocked
hardware. It checks five scenarios: normal injury sequencing and repeated
commands; pause timing; empty re-audit; invalid re-audit;
ordinary/stash compatibility. Upper `tests/test_injury_backoff.py` checks
five handoff and fresh-frame scenarios using the Python standard library.

The older upper `test_first_green_flow_and_navigation_does_not_use_t265_stuck_actions`
already fails at line 124 on the unmodified baseline. Do not describe the
entire historical upper suite as passing. No firmware was flashed, and no
GitHub commit/push was performed by this verification build.

Before ground testing, verify 81/99 degrees really opens both claws and
keeps the injury contained. Inspect 140-degree framing after the timed
reverse, dropped-cargo handling, and the half-close->full-close transition.
