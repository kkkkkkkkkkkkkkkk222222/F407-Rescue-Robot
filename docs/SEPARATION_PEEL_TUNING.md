# Small lateral peel before cargo separation

## Scope

This trial addresses blocks staying wedged inside the claws or multiple blocks
moving together with the chassis. It does not change PID gains, motor drive,
normal grip angles, injury half-close/backoff, audit rules or the three-attempt
limit. Firmware compilation is not physical validation.

## Retained claw

Both selective DISPERSE and unilateral RELEASE start with 10 degrees of extra
closure, replacing their previous 15/25-degree baselines. The initial retained
left/right angles are 66/114 degrees; normal touch remains 76/104 degrees.
The opposite claw still fully opens to 108/72 degrees.

Existing same-side retries still relax by 10 degrees per accepted attempt:
extra closure is 10, then 0, then -10 degrees relative to normal touch. This is
not a fixed 10-degree squeeze on every retry. Empty/illegal outcomes must still
pass through the original audit/recovery logic, never direct transport.

## Motion

- Selective DISPERSE: open the release side and wait the existing 600 ms,
  move toward that side by a nominal 20 mm at 180 mm/s, stop, then reverse
  straight by the existing 300 mm request at up to 500 mm/s.
- RELEASE_LEFT/RIGHT: retain the existing release-completed handoff; its
  following YIELD performs the same 20 mm peel, then the requested backoff.
- Keep left -> peel right; keep right -> peel left. This preserves the physical
  side used by the old curve, accounting for the different lateral API signs.
- The short lateral phase uses existing Motor_MoveAngle heading hold and
  encoder path completion. The reverse phase uses Motor_MoveDistance heading
  hold and retains its progress through PAUSE, with no intentional lateral or
  yaw trajectory. Normal heading corrections may still move the rear wheel.
- The original camera-to-140 and settling/re-audit phases follow the movement.

The 20 mm target is encoder-estimated chassis travel, not a guaranteed physical
hard stop: braking, slip and heading corrections can change actual displacement.
The old continuous lateral 180 mm/s and yaw 100 mm/s curve is removed from these
two separation paths, so it is not added on top of the new small peel.

First-green bump, no-side 20-degree observation, temporary-stash release and
safe-zone sweep trajectories are unchanged. Existing action timeouts, STOP,
ABORT, HOLD cancellation and manual PAUSE semantics remain in effect.

## Upper-computer compatibility

No opcode, wire mode or context format changes. Existing DISPERSE and
RELEASE -> YIELD commands still apply; mode35/mode30 indicate completion only
after the new internal phases and original camera settling are complete.
Do not insert another upper-controlled lateral move, allocate a new action on
each retransmission, or treat the internal phase change as action completion.
Preserve the current post-action audit, class/count rules and three-attempt
fallback. This trial does not introduce adaptive retry direction or vision-based
mid-motion stopping.

Build all three presets; flash NormalRun for the competition flow. No new
self-test program is added or run, and no automatic flashing/motion is performed.
