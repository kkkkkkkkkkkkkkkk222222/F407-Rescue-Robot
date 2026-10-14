# Adaptive curved cargo separation

## Scope

This replaces the fixed 20 mm lateral peel introduced in 96666f9/b0d8875.
It uses the mirrored curve direction from 1e6a5bc, followed by a straight pull
at the resulting heading. PID gains, motor drive, normal grip angles, injury
half-close/backoff and formal transport legality remain unchanged. Parameters
below are an initial hardware trial, not proven physical separation guarantees.

## Retained claw

Each accepted separation round latches its side and grip profile from a fresh
three-frame/STABLE audit. A physical side that remained exactly one eligible
object across the samples gets 20 degrees extra closure, then 15 and 10 on
subsequent same-side/same-profile attempts. Multi-object or ambiguous sides use
10, then 5, then 0. Zero means normal touch, not an open claw.

Eligibility respects first-green and destination rules: first formal cargo is
GREEN; later material cargo is GREEN/CORE; injury destination requires INJURED.
UNKNOWN evidence never authorizes strong hold. A contradictory newer picture
revokes the strong-side mask even when the overall class histogram is stable.
Mode37 now accepts fresh audits and expires stale evidence like other audit modes.

Changing side/profile resets only that profile's relaxation sequence, never the
three-round total. Repeated action packets and RELEASE -> YIELD do not count
twice. Full-claw legality still ends separation and requires normal GRAB/audits.
Mixed multi-object reports retain the existing bounded host-decision window;
an UNKNOWN side cannot be assumed to contain no useful target or be marked legal.

## Motion

- Open the release side and wait the existing 600 ms before selective DISPERSE.
- Arc: reverse up to 300 mm/s, lateral up to 180 mm/s and wheel-yaw tangent up
  to 100 mm/s. Keep-left uses the old physical right/CW curve; keep-right mirrors it.
- The relative heading goals are 12/16/20 degrees for rounds 1/2/3. Arc path
  ceilings are 100/130/160 mm, within the total stroke. After at least 60 mm,
  achieving the heading goal can end the arc early. Path limits take priority.
- Smoothly remove lateral/yaw commands, then latch the resulting heading.
  Pull backward at up to 500 mm/s, decelerating near the stroke end. Do not
  turn back to the starting heading and sweep released blocks again.
- Selective DISPERSE retains the 300 mm total encoder path ceiling; YIELD uses
  its existing requested magnitude. No additional fixed lateral step is added.
- The lateral corridor is measured relative to this round's start pose.
  Use peak absolute deviation, not accumulated absolute rounded x/y steps.
  The configured 60 mm boundary reserves 5 mm for braking: reaching the inner
  55 mm threshold stops the round early for re-audit. Outward combined motion
  slows on approach, including lateral projection of the final straight pull.
- A 25-degree excursion ends outward turning. These are software estimates;
  braking, slip and calibration error can exceed the nominal physical bounds.
- PAUSE preserves the origin, stage, progress and grip, then ramps speed from
  zero on resume. STOP/ABORT still cancel. Complete at mode35/mode30 only after
  the original camera-to-140 settling and fresh audit handoff.

## Third failure recovery

Open both claws, reverse the existing 300 mm recovery distance, close to normal
76/104-degree touch angles, then advertise mode47 with GRIPPER_CLOSED reflecting
the recovery closure. Audit validity and carried-cargo context are cleared.
After RETURN reaches the centre, open the claws before advertising mode3 and
completed RETURN. Recovery closure must never be interpreted as legal cargo.

First-green bump, no-side 20-degree observation, temporary-stash release and
safe-zone sweep trajectories keep their existing behavior.

## Upper-computer compatibility

No opcode or context format changes. The inspected upper revision is
codex/gamepad-teleop@ff6fe77. Its mode47 handler already clears carried context,
and RETURN waits for fresh mode3. Keep those guards, use real per-side counts,
and do not add an upper-controlled lateral move or restart a retransmitted action.
See SEPARATION_ARC_HANDOFF.md for the Chinese integration note.

## Evidence and verification

Design principles, not chassis tuning values, are informed by:

- [Learning Push-Grasping in Dense Clutter](https://robot-clutter.github.io/ppg)
- [Guided Pushing for Object Singulation](https://users.cs.utah.edu/~thermans/papers/hermans-iros2012.pdf)
- [MIT stable prehensile pushing](https://web.mit.edu/mcube/research/inhand.html)

These describe arm/tabletop experiments and include video/project material;
videos were not fully played in this session. This chassis has no contact-force
feedback, so geometric wedging can still require release and another approach.

The existing audit/handshake and injury harnesses cover grip selection, sample
freshness, retry accounting, mirrored trajectories, pause/resume, corridor
quantization, braking and close-return-open recovery. No new test program is
introduced. Build all three presets and flash NormalRun for the competition
flow. No automatic flashing or vehicle actuation is performed.
