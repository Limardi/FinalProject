# Yaw Alignment Change Report

## Summary

The change reduces overshoot during AMCL-based heading alignment in the Auto Full Task flow. The car was rotating too aggressively near the target angle, so the printed `angle_err` could pass through zero and then grow again instead of settling.

No grasp sequence behavior was changed.

## Files Changed

- `pros_car/src/pros_car_py/pros_car_py/auto_task.py`

## What Changed

### 1. Lowered Rotation Speed

Old values:

```python
ROT_MIN = 250.0
ROT_MAX = 300.0
KP_RETURN_ROT = 5.0
```

New values:

```python
ROT_MIN = 200.0
ROT_MAX = 240.0
KP_RETURN_ROT = 3.5
```

Reason:

- The old minimum rotation speed was too high.
- Even a small heading error still produced a strong spin command.
- With Unity/ROS delay, the car could overshoot before AMCL updated.

### 2. Tighter Alignment Deadband

Old value:

```python
HOME_ANGLE_DB = 20.0
```

New values:

```python
HOME_ANGLE_DB = 6.0
NAV_CLOSE_ANGLE_DB = 10.0
```

Reason:

- `20` degrees was too loose for real alignment.
- `6` degrees gives better heading accuracy.
- `10` degrees is used when close to a waypoint so the car does not spin forever near the target.

### 3. Added Pulsed Turning Near Zero

Added:

```python
TURN_FINE_ANGLE = 30.0
TURN_MEDIUM_ANGLE = 70.0
```

Added helper:

```python
_pub_turn_error(angle_err)
```

Behavior:

- `0-30` degrees: rotate only 1 out of every 4 control ticks.
- `30-70` degrees: rotate every other control tick.
- `>70` degrees: rotate continuously.

Reason:

- Unity needs enough RPM to start rotating, but continuous minimum-speed rotation near zero causes overshoot.
- Pulsing lets AMCL and Unity catch up between turn commands.

### 4. Updated AMCL Heading Correction Calls

Changed these paths to use `_pub_turn_error(...)`:

- Bridge yaw fine-tuning
- Return/home AMCL navigation heading correction

This keeps the original turn direction logic but makes the final alignment gentler.

## Why The Angle Error Was Getting Worse

The controller had this behavior:

1. Compute `angle_err`.
2. If error is above the deadband, send a rotation command.
3. `_pub_omega()` forced that command to at least `ROT_MIN`.
4. `ROT_MIN` was `250 RPM`, so small errors still caused a strong spin.
5. Unity and AMCL update with delay.
6. The car physically passed the correct angle before the terminal showed the updated pose.
7. The next printed `angle_err` could be larger again, but on the opposite side.

So the issue was not only math. It was also command speed plus delayed feedback.

## Current Tuned Values

```python
ROT_MIN = 200.0
ROT_MAX = 240.0
HOME_ANGLE_DB = 6.0
NAV_CLOSE_ANGLE_DB = 10.0
KP_RETURN_ROT = 3.5
TURN_FINE_ANGLE = 30.0
TURN_MEDIUM_ANGLE = 70.0
```

## Verification Already Done

The file parses correctly:

```text
syntax ok
```

The active `pros_car` container imports the new values:

```text
ROT_MIN 200.0
ROT_MAX 240.0
KP_RETURN_ROT 3.5
```

## How To Test

Inside the `pros_car` container:

```bash
r
ros2 run pros_car_py robot_control
```

Then run Auto Full Task and watch logs like:

```text
[return] dist=... angle_err=...
```

Expected behavior:

- Large angle errors should rotate normally.
- Medium errors should rotate more gently.
- Small errors near zero should pulse and settle instead of overshooting.

## If It Still Overshoots

Lower these values:

```python
ROT_MAX = 220.0
KP_RETURN_ROT = 3.0
```

## If It Does Not Rotate Enough

Increase only the minimum:

```python
ROT_MIN = 220.0
```

Do not increase `ROT_MAX` first, because that can bring back overshoot.

