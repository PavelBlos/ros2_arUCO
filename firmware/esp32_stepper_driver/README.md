# Termit motor control v2

Validated on the robot on 2026-09-08, ESP32-D0WD-V3, 4 MB flash.
Build with Arduino ESP32 core **3.3.11**, FastAccelStepper **1.2.7**,
board `esp32:esp32:esp32`. Version query `v` returns `TERMIT_FASTACCEL_V2`.
The Python API requires this firmware before it sends velocity targets.

Hardware step generation and acceleration are managed by FastAccelStepper.
Default acceleration is 1600 steps/s²; the Python configuration is sent using
`c <steps/s²>`. Default host speed limit remains 1100 steps/s.
Legacy `min_start_speed_steps`, `max_linear_accel` and `max_angular_accel`
configuration fields do not control the firmware ramp.

- `s F R L`: signed target speeds in steps/s, including ramped zero/stop.
- `k`: heartbeat, without changing targets. Host sends it every 200 ms or
  faster for short watchdog settings.
- `stop` / `x`: emergency abort (may lose an in-flight step in odometry).
- `e 1`: abort and remove power; `e 0`: enable shared GPIO21/GPIO17 outputs.
- `a 1`: remove power 2 s after actual stop; `a 0`: hold power.
- `w milliseconds`: communication timeout, followed by controlled braking.
- `r`: reset counters while stationary; `q`: power/running/watchdog status.
- `o pF pR pL sF sR sL`: 20 Hz step counts and current signed speeds.
- `m index steps`: finite ramped move while stationary, heartbeat required
  if the move lasts longer than the configured watchdog.
- `t index speed duration_ms`: timed test, explicit speed in steps/s, duration
  limited to 10 s. The old implicit multiplication of small test speeds is removed.
- `h index direction steps`: finite hardware-generated move, same watchdog
  constraint as `m`; no blocking bit-banging.

Navigation now allows the normal 2 s power timer to finish the braking ramp
at route completion, instead of cutting power after 0.2–0.3 s.

Validation: 7 host unit tests; on-device acceleration, reversal, normal stop,
watchdog stop, emergency stop, disable, exactly 50 steps, and auto-sleep;
live Python API connect/drive/stop/disable; web service restart.
These checks observe commanded/executed electrical steps, not encoder-confirmed
rotor motion. Mechanical smoothness under load still needs observation.

Full original flash and original deployed Python files are backed up on Pi in
`/home/raspberry/arUco_termit/backup-before-fastaccel/`.
Bench telemetry: `/home/raspberry/arUco_termit/fastaccel-validation.json`.
Before any rollback, stop the navigation process and remove motor power.
Restore firmware and Python API together; v2 API refuses v1 firmware.

Library reference: https://github.com/gin66/FastAccelStepper
