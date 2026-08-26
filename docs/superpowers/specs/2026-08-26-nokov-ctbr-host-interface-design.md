# Nokov State and CTBR Host Interface

## Scope

Add a host-side ROS interface for using Nokov rigid-body measurements in an
external Crazyflie controller. The controller itself is intentionally out of
scope: this change only publishes measured state and accepts CTBR commands.

The Crazyflie firmware already flashed on the aircraft is not modified or
reflashed. The host uses its existing legacy RPYT CRTP command path with the
firmware's existing rate-mode parameters.

## Interfaces

Each configured Crazyflie publishes:

```
/cf<ID>/mocap_state  crazyswarm/MocapState
```

`MocapState.msg` is:

```
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acceleration
bool valid
bool derivatives_valid
```

`header.frame_id` is `world`. Position, orientation, linear velocity, and
linear acceleration are expressed in `world`; `twist.angular` is the angular
velocity of the Crazyflie body, expressed in the Crazyflie body frame. The
linear quantities use metres, seconds, and SI derivatives. Angular velocity
uses radians per second.

`valid` is true only for a tracked Nokov rigid body. `derivatives_valid` stays
false until the estimator has enough consecutive frames to produce velocity,
angular velocity, and acceleration. A long gap or invalid time interval resets
the derivative state.

The rigid-body definition must be aligned with the Crazyflie body axes before
flight. The initial implementation assumes that alignment; a restrained bench
test must verify the roll, pitch, and yaw signs before nonzero CTBR commands
are allowed in flight.

Each configured Crazyflie also subscribes to:

```
/cf<ID>/cmd_ctbr  crazyswarm/CTBR
```

`CTBR.msg` is:

```
std_msgs/Header header
geometry_msgs/Vector3 body_rates
float32 collective_thrust
```

`body_rates` is `[p, q, r]` in rad/s and `collective_thrust` is total vehicle
thrust in N. For the current aircraft, `ctbr_max_thrust_newton` defaults to
`1.176798` N (120 gf total static thrust). The host converts the command to
the firmware's legacy rate command in deg/s and its raw thrust range
`0..60000`.

## Data Flow

```
Nokov -> libmotioncapture -> CrazyflieGroup::runFast()
      -> /cf<ID>/mocap_state -> future external controller
      -> /cf<ID>/cmd_ctbr -> CrazyflieROS -> legacy RPYT CRTP packet
      -> existing onboard rate controller and motor mixer
```

The existing `crazyswarm_server` Nokov connection remains the only connection
used for flight. `mocap_helper` remains a console debugging utility and is not
started by `hover_swarm.launch`.

The measured-state publisher belongs to each `CrazyflieROS` object and is
called from `CrazyflieGroup::runFast()` immediately after its matching rigid
body is available. This keeps the rigid-body association and per-vehicle
derivative history local to one object.

## Derivative Estimation

The first implementation derives values from consecutive valid mocap frames:

- Linear velocity is position difference divided by frame interval.
- Body angular velocity is quaternion delta converted to an axis-angle vector
  and divided by frame interval.
- Linear acceleration is the difference of filtered linear velocity divided by
  frame interval.
- A configurable first-order low-pass filter is applied to derivative values.

Frame receipt time is used as the initial timestamp source. This avoids relying
on undocumented units of Nokov's `iTimeStamp`. Invalid, non-positive, or
excessively large intervals reset history rather than creating a derivative
spike.

## CTBR Bridge and Safety

`CrazyflieROS` gets a dedicated `cmd_ctbr` subscriber; `/cmd_vel` retains its
current RPYT semantics and is not reused. The bridge rejects non-finite input,
clamps body rates and collective thrust using ROS parameters, converts units,
and forwards the result through the existing `Crazyflie::sendSetpoint()`.

The launch configuration sets existing firmware parameters at runtime:

```
flightmode.stabModeRoll: 0
flightmode.stabModePitch: 0
flightmode.stabModeYaw: 0
```

`0` means RATE in the flashed firmware. Crazyswarm writes these through the
radio parameter interface when the server starts; no firmware flash is needed.
The bridge sends a zero-thrust packet before accepting nonzero CTBR thrust, as
required by the legacy commander thrust lock. It maps the physical thrust range
to `0..60000`; this linear model is a temporary conservative mapping and can
later be replaced by a voltage-aware thrust calibration without changing the
controller API.

Streaming CTBR takes priority over high-level commands such as `takeoff()` and
`goTo()`. A future controller must publish continuously and issue
`notify_setpoints_stop()` before returning to high-level mode.

## Controller Boundary

No controller source file, launch node, trajectory generation, or automatic
CTBR publisher is included. A later controller only needs to subscribe to
`/cf<ID>/mocap_state` and publish `/cf<ID>/cmd_ctbr` at its chosen control
frequency.

## Verification

- Build the catkin workspace and confirm generated messages are available.
- Launch with the radio disconnected and verify message/topic registration.
- With Nokov connected, check state topic timestamps, pose, derivative validity,
  and derivative units while moving a tracked rigid body by hand.
- Verify CTBR input validation and saturation in unit tests. Hardware CTBR
  flight is not part of this change and requires a separate bench and tethered
  test procedure.
