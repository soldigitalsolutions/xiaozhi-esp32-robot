# Xingzhi Cube 1.54 Robot

This board variant keeps the original Xingzhi Cube 1.54-inch display, audio,
buttons, and power management, and adds three servos, an L298N dual motor
driver, and BLE UART control.

## Wiring

| Function | ESP32-S3 GPIO |
| --- | ---: |
| Left arm servo signal | 41 |
| Right arm servo signal | 1 |
| Head servo signal | 2 |
| L298N IN1 | 17 |
| L298N IN2 | 18 |
| L298N IN3 | 47 |
| L298N IN4 | 48 |

Use a separate suitable power supply for the motors and servos, and connect its
ground to ESP32-S3 ground. Never power the motors or servos from an ESP32 GPIO.
For the initial motor tests, install the L298N ENA and ENB jumpers so both
channels are enabled at fixed speed. Remove the jumpers only if ENA/ENB are
later connected to GPIOs for PWM speed control.

GPIO 41 is an ESP32-S3 JTAG pin; do not use an external JTAG debugger while the
left arm servo is connected there. Do not move the motor input back to GPIO 20:
it is also used by the ESP32-S3 native USB interface.

## BLE control

Connect to **Xiaozhi-Robot** with an Android BLE UART terminal such as
Serial Bluetooth Terminal. Select the Nordic UART Service when prompted.
Commands are plain text writes:

- `S:L:90`, `S:R:90`, `S:H:90` — smoothly move the left arm, right arm, or
  head to the requested angle. BLE servo moves take about 700 ms and are
  restricted to 45–135 degrees for initial calibration.
- `A:F`, `A:B`, `B:F`, `B:B` — pulse motor output channel A or B forward or
  backward for 400 ms. Channel A is the left wheel; channel B is the right
  wheel. Firmware compensates for the right motor's reversed wiring.
- `M:F`, `M:B` — pulse both motor channels forward or backward for 400 ms.
- `M:S` — stop both motor channels immediately.

## Voice assistant controls

The assistant can call these local MCP tools:

- `self.robot.greet`: alternate both arms in a short, smooth three-cycle
  greeting, then return them to center. This runs only when requested; the
  robot remains still at startup.
- `self.robot.look`: turn the head left (`direction=-1`), right (`1`), or back
  to center (`0`); left/right movements return to center automatically.
- `self.robot.drive`: set left/right wheel direction (`-1` reverse, `0` stop,
  `1` forward) for 100–1000 ms. Every pulse stops automatically. For longer
  movement, the assistant should issue brief pulses and listen between them;
  `self.robot.stop` stops the wheels immediately and clears queued movement.
- `self.robot.move_servo`: move one servo (0 left arm, 1 right arm, 2 head)
  smoothly to 45–135 degrees over 300–2000 ms.
- `self.robot.stop`: stop both wheels and discard queued robot commands.

The board has no wheel encoders or distance sensor. A request such as "advance
one meter" can only be estimated from timed pulses, not measured accurately;
calibrate the distance on the floor before relying on such requests.

Use these for brief, conversational movements rather than continuous driving.
A positional servo continues holding its last position while powered; this is
normal and can cause a faint holding sound. A servo should not keep sweeping
after a single command.

The greeting initially moves the arm servos between 75 and 105 degrees and
returns them to 90 degrees. Head turns use 70 and 110 degrees. These are
conservative starting angles; test with the arms clear and adjust the angles
after confirming each servo's physical orientation. Test with the wheels lifted
and the servo arms clear of obstacles first.

The BLE UART characteristic is currently unauthenticated. Keep the robot's
motor power off until the trusted phone is connected and the controls are
ready; nearby BLE devices could otherwise issue movement commands.
