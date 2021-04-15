import math
import math3d
import servo
import pid
import ahrs.madgwick

/**
Gimbal stabilizer over two angles of freedom, controlled as two servo motors.

The stabilizer uses the Madgwick AHRS algorithm to compute the absolute orientation,
  which in term is feeded to a PID Controller, tuned to rapid feedback with
  no overshoot.
*/
class Stabilizer:
  static APPLY_RATE   ::= Duration --ms=20  // 50hz.
  static RAD_PER_DEG  ::= math.PI / 180.0

  // PID tuning using the Ziegler–Nichols method, with "no overshoot".
  // See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method.
  static KU ::= 200.0
  static TU ::= 0.08

  static KP ::= 0.2 * KU
  static KI ::= 0.4 * KU / TU
  static KD ::= 0.066 * KU * TU

  pitch_/servo.Motor
  roll_/servo.Motor

  pitch_pid_ ::= pid.Controller --kp=KP --ki=KI --kd=KD --min=-90.0 --max=90.0
  roll_pid_ ::= pid.Controller --kp=KP --ki=KI --kd=KD --min=-90.0 --max=90.0

  madgwick_ ::= ahrs.Madgwick

  last_/int := Time.monotonic_us
  next_apply_/int := Time.monotonic_us

  constructor --pitch/servo.Motor --roll/servo.Motor:
    pitch_ = pitch
    roll_ = roll

  /**
  Updates the stabilizer with new accelerometer and gyroscope measurements.

  The servo motors are updates as well, when applicable.
  */
  update accel/math3d.Vector3 gyro/math3d.Vector3:
    start := Time.monotonic_us

    elapsed := Duration --us=start - last_

    madgwick_.update_imu
      gyro * RAD_PER_DEG
      accel
      elapsed

    if start >= next_apply_:
      euler_angles := madgwick_.rotation.euler_angles

      // Compute new pitch and roll angles for the servo motors.
      pitch := euler_angles.y
      pitch_degrees := pitch_pid_.update -pitch APPLY_RATE

      roll := euler_angles.x
      roll_degrees := roll_pid_.update roll APPLY_RATE

      // Update the servo motors with the new angles.
      pitch_.degrees = pitch_degrees
      roll_.degrees = roll_degrees

      next_apply_ += APPLY_RATE.in_us

    last_ = start
