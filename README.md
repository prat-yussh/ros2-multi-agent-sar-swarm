# ROS 2 Multi-Agent Search & Rescue Swarm

A 3-robot autonomous swarm in ROS 2 + Gazebo that searches a 30×30m disaster-hospital
environment, locates a victim using LiDAR + odometry only (no SLAM, no cameras), and
coordinates a return to a safe zone.

## Fleet

| Robot | Role | Color | Search Lane (X) |
|---|---|---|---|
| `robot_1` | Leader / Center | Blue | -5.0 to 5.0 |
| `robot_2` | East Flank | Green | 5.0 to 14.0 |
| `robot_3` | West Flank | Yellow | -14.0 to -5.0 |

**Sensors:** 2D LiDAR (`/scan`) and wheel odometry (`/odom`) only.
**World:** `disaster_hospital.sdf` — 30×30m room, safe zone at Y: 5–12, search room at
Y: -15–5, connected by a 2m-wide doorway gap (X: -1 to 1) at Y≈5.

## Architecture

    sar_swarm_ws/src/
    ├── sar_bringup/launch/
    │   ├── hospital_world.launch.py   # Gazebo + world + spawns all 3 robots
    │   ├── spawn_robot.launch.py      # Per-robot spawn template (x/y/yaw/name/color)
    │   └── swarm_logic.launch.py      # Launches 3x robot_controller (one per robot)
    ├── sar_gazebo/worlds/
    │   └── disaster_hospital.sdf
    └── sar_logic/sar_logic/
        └── robot_controller.py        # Finite state machine — one process per robot

Each robot runs its own independent `robot_controller` process, parameterized by
`robot_name`. There's no central coordinator — robots share state only via two topics:
`/victim_location` (victim coordinates, broadcast by the leader once found) and
`/rescue_alert` (a `Bool` that tells all three to transition into `RESCUE`).

## Coordinate frame

All three robots spawn at `yaw: -1.57`. `odom_callback` rotates each robot's raw
odometry into a shared room-aligned frame:

    self.current_y = self.start_y - odom_x
    self.current_x = self.start_x + odom_y

This is only valid because every robot shares that same spawn yaw — verified against
`hospital_world_launch.py` early on, and re-verified directly against Gazebo ground
truth later (see Known Limitations). `current_x` is signed so that positive = East,
matching `robot_2`'s `start_x = +0.7`.

## Search pattern

A lawnmower sweep, bounded per-robot to a disjoint lane (`lane_min_x`/`lane_max_x`) so
the three robots cover distinct thirds of the room in parallel rather than converging
on the same territory:

1. **DEPLOY** — march south from the safe zone to the room entrance.
2. **FAN_OUT_TURN / FAN_OUT_DRIVE** — flank robots turn and drive directly to their
   lane center (`(lane_min_x + lane_max_x) / 2`); the leader holds center.
3. **MARCH** — sweep north/south until a wall or the `Y=4.5` safe-zone geofence is hit.
4. **SWEEP_TURN_1 / SWEEP_SHIFT / SWEEP_TURN_2** — 180° lawnmower turn, shifting
   laterally within the robot's own lane bounds (reversing shift direction on hitting
   either the real wall or the lane edge).

The lane-center fan-out (rather than incremental lap-by-lap drift toward it) was the
key fix for search speed — see Debugging Notes.

## Victim spawner

The leader (`robot_1`) spawns a visual-only marker — no `<collision>` element, so LiDAR
passes through it — at a random point inside the search room:

    subprocess.Popen([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-string', sdf, '-name', victim_name,
        '-x', str(self.victim_x), '-y', str(self.victim_y), '-z', '0.3'
    ])

Two details that mattered in practice:
- **Position must be passed via `-x`/`-y`/`-z` flags**, not just embedded in the SDF's
  `<pose>` tag — the latter was silently ignored by `ros_gz_sim create` in testing.
- **The model name must be unique per spawn** (`f"victim_{int(time.time()*1000)}"`) —
  a fixed name causes `ros_gz_sim create` to silently no-op against an existing model
  from a prior run, leaving an invisible "ghost" victim at the old location while the
  code reports new coordinates.

Coordinates are broadcast on a repeating 1s timer (`/victim_location`), not a single
publish — otherwise any robot that starts subscribing after the one-shot message would
never receive a real position.

## Detection: math + LiDAR fusion

Victim detection isn't purely coordinate-based. Accumulated odometry drift after many
sweep turns can leave a robot's internal position meters from Gazebo ground truth, so
detection is fused with a LiDAR contact check as a hard failsafe:

    victim_by_math = dist_to_victim < 2.0
    victim_by_contact = self.min_front < 1.0 and self.state in [
        "MARCH", "SWEEP_TURN_1", "SWEEP_SHIFT", "SWEEP_TURN_2"
    ]

`victim_by_contact` is scoped to active search states only, so it can't misfire during
`RESCUE` or fan-out. In testing, this caught a real case where accumulated drift (~0.7m
after dozens of 180° turns) put the math-only check just outside its 2.0m radius while
the robot was physically touching the marker.

## Rescue

On detection, the finder publishes `/rescue_alert`; all three robots switch to
`RESCUE` and drive a per-robot waypoint sequence back to the safe zone. Flank robots
retreat straight up their own lane's X first, and only pinch toward the doorway's ±0.7m
offsets once near the gate — a single diagonal waypoint from deep in each flank lane
caused robots to cross paths and collide near the door. A stuck-breaker (reverse, then
turn, if blocked past ~2s) prevents robots from spinning in place when LiDAR sees
another robot at close range.

## Known limitations

- **Odometry drift is tolerated, not corrected.** No SLAM/localization fusion — this
  is by design per the sensor spec, but it means `current_x/current_y` accuracy
  degrades with sweep-turn count. The LiDAR-fusion failsafe covers this for detection;
  it does not correct drift for `RESCUE` waypoint navigation.
- **No inter-robot collision avoidance during search**, only during `RESCUE`. Lane
  bounds keep robots spatially separated during normal sweeps, so this hasn't caused
  issues in testing, but it's not an explicit guarantee.
- **`disaster_hospital.sdf` wall positions are hardcoded** into lane bounds and
  waypoints (e.g. `lane_max_x = 14.0`, doorway offset `±0.7`) rather than read from the
  world file — changing the world requires updating these constants in
  `robot_controller.py` to match.
