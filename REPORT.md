# Path-Following Controller

## 1. Overview

The controller follows the recorded path with pure pursuit. It splits the path into forward and reverse legs, builds a speed profile for each leg, and sends body-frame speed and yaw-rate setpoints to ArduRover in GUIDED mode. The low-level speed and steering loops stay in ArduRover.

The code is in:

- [`src/ardurover_nav/src/ardurover_controller.cpp`](src/ardurover_nav/src/ardurover_controller.cpp)
- [`src/ardurover_nav/include/ardurover_nav/ardurover_controller.hpp`](src/ardurover_nav/include/ardurover_nav/ardurover_controller.hpp)

Outside the controller I changed one line in the `Dockerfile`: the pip step now pins `"numpy<2"`. Without it a clean image build fails, because the current MAVProxy pulls an OpenCV build that needs numpy 2, and pip can't replace the numpy 1.26 that Ubuntu installs.

## 2. Arming and GUIDED mode

I kept the original setup sequence and fixed two problems that showed up in SITL. In both cases the rover stayed in MANUAL and ignored every setpoint.

1. Until MAVROS gets the first heartbeat it can't map a mode name to an ArduPilot mode, so `/mavros/set_mode` returns `mode_sent = false`. The original code went on to arming anyway. Now the GUIDED request is repeated once per second until it is sent.
2. ArduRover already sends heartbeats while it is still booting (mode `INITIALISING`). A GUIDED request and an arm command accepted in that window are undone when the boot finishes, and the rover comes up in MANUAL and disarmed. The controller now subscribes to `/mavros/state` and sends GUIDED only after the vehicle has left `INITIALISING`.

Tracking starts only when `/mavros/state` reports GUIDED and armed. If that doesn't happen within 5 s, or the rover later leaves GUIDED or gets disarmed, the setup sequence runs again.

Only body-frame speed and yaw rate are sent, so a small mismatch between the EKF heading and the Gazebo ground truth doesn't affect tracking.

## 3. Commands sent to ArduRover

`trajectory_controller_node` calls `Control()` at 20 Hz with the latest `/ground_truth/odom`. Every call publishes one message:

| Field | Value |
|---|---|
| Topic | `/mavros/setpoint_velocity/cmd_vel_unstamped` |
| Type | `geometry_msgs/msg/Twist` |
| `linear.x` | speed `v` in m/s, negative on reverse legs |
| `angular.z` | yaw rate `ω` in rad/s, counter-clockwise positive (REP-103) |
| other fields | 0 |

During setup the controller sets `mav_frame = BODY_NED` on the MAVROS `setpoint_velocity` plugin. MAVROS then converts the vector from FLU to FRD and sends `SET_POSITION_TARGET_LOCAL_NED` with `type_mask = 0x5C7`, which tells ArduRover to ignore position, acceleration and yaw and to use velocity and yaw rate.

ArduRover 4.6 handles this in `ModeGuided::set_desired_turn_rate_and_speed()`:

- speed: the target is `sign(vx)·|v|`, clamped to `WP_SPEED` (1.5 m/s) and tracked by the speed PID (`ATC_SPEED_*`) with the accel limit `ATC_ACCEL_MAX`;
- turn rate: the target is `−ω` (NED is clockwise positive), tracked by the rate PID (`ATC_STR_RAT_*`) and limited by `ATC_STR_RAT_MAX` and `ATC_STR_ACC_MAX`;
- the skid-steer mixer turns both into left and right wheel outputs.

If no setpoint arrives for 3 s, GUIDED stops the rover. After the last leg the controller keeps publishing zeros, so the rover stays still.

## 4. Path preprocessing

All of this runs once, in the constructor.

### 4.1. Removing duplicate points

Waypoints closer than 5 cm to the previous kept point are dropped. Most of them were recorded while the rover was standing still, and without them every segment has a well-defined direction. The cumulative arc length `s_i` is stored for each remaining waypoint.

### 4.2. Driving direction

A segment was driven in reverse if it points against the recorded heading:

```
(p[i+1] − p[i]) · (h(ψ[i]) + h(ψ[i+1])) < 0,    h(ψ) = (cos ψ, sin ψ)
```

### 4.3. Splitting into legs

Consecutive segments with the same direction form one leg. A direction change starts a new leg only if it lasts at least 0.3 m. Shorter reversals are recording noise and stay in the current leg, and a short reversed tail at the very end of the path is cut off. For the supplied paths this gives:

- `0-drive-straight`: forward, then about 1 m in reverse;
- `1-drive-with-turns`: forward, then about 0.7 m in reverse;
- `2-complicated`: forward, about 1.1 m in reverse (a three-point turn), forward again, then about 0.3 m in reverse.

### 4.4. Speed profile

At each waypoint B the curvature is estimated from the circle through the points A and C that lie 0.5 m before and after it along the leg (Menger curvature). The speed limit then follows from the lateral acceleration limit:

```
κ_i = 2·|AB × BC| / (|AB|·|BC|·|AC|)
v_i = min(v_max, sqrt(a_lat / κ_i))
```

The speed at the end of each leg is set to 0, and a backward pass adds braking, so the rover slows down before tight turns and stops at every cusp:

```
v[i−1] = min(v[i−1], sqrt(v[i]² + 2·a_dec·(s[i] − s[i−1])))
```

with `v_max = 1.5 m/s`, `a_lat = 0.45 m/s²` and `a_dec = 0.5 m/s²`.

## 5. Control step

### 5.1. Progress

Every cycle the controller looks for the closest point on the current leg, starting from the current segment and going forward within a 1.5 m window. That gives the progress `s` and the cross-track error. The search never goes back and never looks at other legs. On `2-complicated` this matters: the path crosses itself, and after the three-point turn it runs on top of the earlier track, so a global nearest-point search would jump to the wrong part of the path.

### 5.2. Lookahead

```
v_plan = profile(s + v_meas·t_r),        t_r = 0.5 s
L      = clamp(T_L·v_plan, 0.6 m, 1.5 m),  T_L = 1.2 s
```

`t_r` looks a bit ahead in the profile to cover the rover's response lag. The target is the point at arc length `s + L` on the leg. Near the end of a leg it is extrapolated past the last waypoint along the last segment, which keeps the steering calm on the final approach.

The lookahead uses the planned speed, not the measured one, on purpose. With the measured speed, a big correction slows the rover down, the lookahead gets shorter and the gain goes up. Together with the turn-acceleration limit this can end up in a sustained weave.

### 5.3. Pure pursuit

The heading used for tracking is `θ = ψ` on forward legs and `θ = ψ + π` on reverse legs. When reversing, the rear of the rover acts as a virtual front, so the same law works in both directions. With `(dx, dy) = target − position`:

```
x_l =  cos θ·dx + sin θ·dy
y_l = −sin θ·dx + cos θ·dy
α   = atan2(y_l, x_l)
κ   = 2·y_l / (x_l² + y_l²)              (= 2·sin α / d)
v   = max(min(v_plan, sqrt(a_lat / |κ|)), v_min)
ω   = clamp(v·κ, −ω_max, ω_max)
cmd = (±v, ω)                            (minus on reverse legs)
```

with `v_min = 0.15 m/s` and `ω_max = 1.2 rad/s`. The curvature doesn't depend on the direction of travel, and for the virtual vehicle `dθ/dt = dψ/dt`, so the same `ω` is correct in reverse.

### 5.4. Cross-track and heading error

Pure pursuit doesn't use the errors explicitly, but on a straight section it works out to a PD law. Let `e_y` be the lateral offset from the path and `e_ψ` the angle between the rover heading and the path direction. For small errors `y_l ≈ −e_y − L·e_ψ`, so

```
ω ≈ −(2v/L²)·e_y − (2v/L)·e_ψ
```

The cross-track error is the P term and the heading error acts as its derivative, since `ė_y ≈ v·e_ψ`. The closed loop is `ë_y + (2v/L)·ė_y + (2v²/L²)·e_y = 0`. With `L = T_L·v` that gives `ω_n = √2/T_L` and `ζ = 1/√2` at any speed.

If ArduRover's turn-rate response is modelled as a first-order lag `τ`, the characteristic polynomial becomes `τs³ + s² + (2v/L)s + 2v²/L²`, which is stable only for `T_L > τ`. `T_L = 1.2 s` leaves a wide margin over the inner-loop lag and the 120°/s² turn-acceleration limit.

### 5.5. Turning in place

A skid-steer rover can turn on the spot. If `|α| > 0.8 rad`, the controller sends `v = 0` and `ω = clamp(1.5·α, −ω_max, ω_max)` until `|α| < 0.2 rad`. The two thresholds keep it from switching back and forth.

On the supplied paths the speed profile handles almost all corners. The pivot is there for large heading errors: a start that isn't aligned with the path, arriving at a cusp with the body turned away from the next leg, or a corner the rover couldn't follow in time. In those cases it turns on the spot instead of swinging a wide arc off the path.

## 6. Stall detection and recovery

This part needed the most work. On `2-complicated` the rover sometimes got stuck around s ≈ 46.5 m and stayed there until the scorer timed out.

The cause is in the Baylands world. The dirt path is a separate mesh that lies 2–7 cm above the grass, and its edge is an open step. The recorded path runs along that edge from s ≈ 38 m to s ≈ 46 m with the right wheels on it, then turns right across it at a grazing angle. Ground truth from SITL showed the right wheels within 2 cm of the edge and the heading parallel to it within 0.3°. The edge blocks the yaw to the right, ArduRover's turn-rate integrator winds up until the mixer holds the right wheels at zero (left side full, right side 1500 µs), and the rover stops dead at about (42.6, −26.1). The husky wheels (`mu 10`, `mu2 5`, at most 40 N·m per joint) can't climb the step sideways. Whether it gets stuck depends on how the rover meets the edge: in some runs it bounced over at a small angle and drove on.

The controller counts it as a stall when the rover isn't turning in place and its measured speed stays below 30 % of the commanded speed, or below 0.05 m/s if that is higher, for 1 s. Detection only starts after the rover has moved once. Before the EKF origin is set, ArduRover accepts GUIDED and arming but keeps the rover still, and that wait isn't a stall.

Recovery has three steps:

1. Turn in place towards a point 1 m further along the path than the normal lookahead, until `|α| < 0.05 rad`. Each further stall within 1.5 m of the previous one adds another metre, so every attempt meets the edge at a steeper angle.
2. Send zero until the rover has been still for 0.5 s (at most 4 s). While the rover is stuck, ArduRover's speed integrator winds up as well, and its stop logic only clears it when the vehicle is stationary. Without this pause the rover jumped to 1.6–1.7 m/s as soon as it broke free and overshot the bend by 0.7–0.9 m. I also tried holding the heading during the pause instead of sending zero, but then the rover started hunting on the edge and stalled four times in a row.
3. Drive at no more than 0.5 m/s for the next 1.5 m.

In SITL the rover slides 1–1.5 m back along the edge while it turns. During the pause the steering integrator, still wound towards the blocked side, turns it a bit further, and the normal pivot then brings it back. It leaves at about 20° to the edge and climbs it at 0.5–0.8 m/s on the first try. The recovery costs about 10 s, and the largest cross-track error around it is 0.3–0.4 m.

## 7. Leg switching and goal

A leg is done when less than 5 cm of arc length is left. The projection clamps at the end of the leg, so an overshoot also counts. On that cycle the controller sends a zero command, and the next leg starts in its own direction. After the last leg it logs `Path complete` and keeps sending zeros.

## 8. Parameters

| Constant | Value | Meaning |
|---|---|---|
| `kMaxSpeed` | 1.5 m/s | `v_max`, same as `WP_SPEED`, the GUIDED speed clamp |
| `kMaxLateralAccel` | 0.45 m/s² | `a_lat`, sets the speed in curves |
| `kMaxDecel` | 0.5 m/s² | `a_dec`, below `ATC_ACCEL_MAX` = 1 m/s² |
| `kMinSpeed` | 0.15 m/s | `v_min`, keeps the rover moving until the end of each leg |
| `kLookaheadTime` | 1.2 s | `T_L` |
| `kLookaheadMin` / `kLookaheadMax` | 0.6 / 1.5 m | lookahead limits |
| `kResponseTime` | 0.5 s | `t_r`, profile preview |
| `kMaxYawRate` | 1.2 rad/s | `ω_max`, below `ATC_STR_RAT_MAX` = 120°/s |
| `kPivotEnterAngle` / `kPivotExitAngle` | 0.8 / 0.2 rad | pivot thresholds |
| `kPivotGain` | 1.5 1/s | yaw rate per radian of `α` while pivoting |
| `kMinWaypointSpacing` | 5 cm | duplicate point filter |
| `kMinLegLength` | 0.3 m | shortest reversal that becomes its own leg |
| `kCurvatureBase` | 0.5 m | distance from B to A and C for the curvature |
| `kSearchWindow` | 1.5 m | progress search window |
| `kArrivalTolerance` | 5 cm | leg completion |
| `kStallSpeed` / `kStallSpeedRatio` / `kStallTicks` | 0.05 m/s / 0.3 / 20 ticks (1 s) | stall detection |
| `kUnstickExitAngle` | 0.05 rad | pivot exit after a stall |
| `kRecoveryReach` | 1 m | extra target distance per stall in a row |
| `kRestTicks` / `kMaxSettleTicks` | 10 / 80 ticks | pause: still for 0.5 s, at most 4 s |
| `kRecoverySpeed` / `kRecoveryDistance` | 0.5 m/s / 1.5 m | speed cap after a stall; the distance also decides whether a stall counts as a repeat |
| `kConfirmTicks` | 100 ticks (5 s) | timeout for GUIDED + armed |

While driving, the node logs progress, cross-track error and speed every 2 s.

## 9. SITL results

These are runs of the final code with `control.launch.py gz_gui:=false` on my laptop, where Gazebo ran at about 0.7× real time. Every run ended with `Path complete`. Values from separate runs are separated by `/`.

| Path | Score | Completion | RMS CTE | Max CTE |
|---|---|---|---|---|
| `0-drive-straight` | 92.9 / 94.1 | 0.972 / 0.971 | 0.017 / 0.013 m | 0.094 / 0.059 m |
| `1-drive-with-turns` | 84.4 / 85.6 | 0.975 / 0.976 | 0.050 / 0.050 m | 0.307 / 0.260 m |
| `2-complicated`, supplied scorer | 52.0 / 55.6 / 51.7 | 0.656 / 0.654 / 0.655 | 0.107 / 0.078 / 0.106 m | 0.357 / 0.228 / 0.383 m |
| `2-complicated`, second scorer with `goal_radius_m:=0.3` | 78.4 / 79.8 / 78.3 | 1.000 / 1.000 / 1.000 | 0.112 / 0.097 / 0.111 m | 0.375 / 0.384 / 0.386 m |

The supplied scorer ends `2-complicated` early at a crossing in the middle of the path (see section 10), so the second-scorer row is the one that covers the whole run.

On `2-complicated` the rover stalled at the dirt-path edge in two of the three runs and recovered on the first attempt. In the third run it crossed the edge without stalling. With my first version of the recovery (only turning in place) the second scorer gave 62.5–66.6 and a max cross-track error of 0.70–0.94 m. Now the largest error is about 0.38 m, in the tight turns before the three-point turn at s ≈ 84 m and, in one run, at the stall.

Scores change by a few points between runs. On `1-drive-with-turns` the same tracking code scored between 79 and 90 on this laptop. The worst run had two "target not received" stops from ArduRover caused by CPU load. In the other runs the largest error is at the first bend: after the wait for the EKF origin, ArduRover overshoots to about 2 m/s before its speed loop settles.

Video: [screen recording of a `1-drive-with-turns` run](https://drive.google.com/file/d/1bbLpXDf4Buw4kgGqj1DlCfivCNkJRGV4/view?usp=sharing) (score 89.0), on Google Drive.

## 10. Scorer note for `2-complicated.path`

Around 65 % of its length, near (33.4, −29.7), `2-complicated` passes 0.43 m from its own final waypoint, crossing the final approach. About 1.8 m of that pass lies inside the 1 m goal radius. The scorer samples at 10 Hz and stops after 10 consecutive samples inside the radius. At 1.5 m/s the rover needs about 1.2 s for that stretch, so the scorer can end the run there with `reason: goal_reached` and `goal_reached: false` while the rover keeps driving.

The `completion` written in that case depends on which side of the crossing the nearest-segment projection picks for a few samples, so it comes out as ≈0.65 or ≈0.99 for practically the same trajectory. In my runs it stopped there every time, with ≈0.65 in most runs and ≈0.99 in a few, while the rover went on to `Path complete`.

I left the scorer unchanged. To score the whole run, a second scorer with a smaller goal radius can run next to `control.launch.py` in another container shell:

```bash
ros2 run ardurover_nav path_scorer_node --ros-args \
  -p path_file:=/home/developer/ardurover_navigation/paths/2-complicated.path \
  -p output_file:=/home/developer/ardurover_navigation/paths/score_full.txt \
  -p goal_radius_m:=0.3
```

On the crossing the rover stays at least 0.43 m from the final waypoint, and it ends within 0.03 m of it.

On `0-drive-straight` and `1-drive-with-turns` the final waypoint is about 1 m and 0.7 m behind the farthest point, because both recordings end by reversing. The scorer stops about 1 s after the rover enters the goal radius, so `completion` there tops out around 0.98.
