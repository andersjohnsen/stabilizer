import math
import math3d
import gpio
import gpio.pwm
import serial.protocols.i2c as i2c
import servo
import icm20948

import .stabilizer

MEASURE_RATE ::= Duration --ms=5  // 200hz.

SDA ::= gpio.Pin 21
SCL ::= gpio.Pin 22

PITCH ::= gpio.Pin 18
ROLL  ::= gpio.Pin 19

main:
  // Setup servo using a single PWM controller.
  pwm := gpio.Pwm --frequency=servo.Motor.DEFAULT_FREQUENCY
  pitch := servo.Motor PITCH --pwm=pwm
  roll := servo.Motor ROLL --pwm=pwm

  // Reset the servos to center position.
  pitch.degrees = 0
  roll.degrees = 0
  sleep --ms=1000

  // Setup the ICM20948 IMU driver.
  bus := i2c.Bus --sda=SDA --scl=SCL
  device := bus.device icm20948.I2C_ADDRESS_ALT
  imu := icm20948.Driver device

  // Enable both accelerometer and gyroscope.
  imu.on
  imu.configure_accel
  imu.configure_gyro --scale=icm20948.GYRO_SCALE_1000DPS

  // Initialize the stabilizer.
  stabilizer := Stabilizer --pitch=pitch --roll=roll

  // Run the measure loop.
  while true:
    // Read IMU data.
    accel := to_vector3 imu.read_accel
    gyro := to_vector3 imu.read_gyro

    // Update stabilizer.
    stabilizer.update accel gyro

    // sleep until next measure tick.
    sleep MEASURE_RATE

to_vector3 p/math.Point3f -> math3d.Vector3:
  return math3d.Vector3 p.x p.y p.z
