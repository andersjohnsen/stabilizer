# Gimbal Stabilizer written in Toit

Gimbal stabilizer using IMU, servo motors and a PID controller.

It's written in Toit and can run on any ESP32.

Check out [toit.io](https://toit.io/) for more information about Toit.

## Running it

In `main.toit` the GPIO pins used can be changed:

```
SDA ::= gpio.Pin 21
SCL ::= gpio.Pin 22

PITCH ::= gpio.Pin 18
ROLL  ::= gpio.Pin 19
```

With that in place, it's just a matter of running it on your device:

```
$ toit run -d <my-device> main.toit
```

To stop the stabilizer, press `Ctrl-C`.

## Demo

You can see the code running on an ESP32 here:

[![Demo](https://img.youtube.com/vi/oubwb3fsU4E/0.jpg)](https://youtu.be/oubwb3fsU4E)

The IMU used is ICM-20948, together with two basic servo motors:

* [ICM-20948](https://www.sparkfun.com/products/15335)
* [Servo motors](https://www.sparkfun.com/products/9065)
