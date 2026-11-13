# Observatory-Controller-V2

Arduino firmware for the second iteration of my observatory dome azimuth
controller. It speaks the MaxDome II serial protocol to the computer, drives
the dome rotation motor through an H-bridge, reads a hall-effect encoder on
the drive wheel, and takes a magnetometer heading by radio from a separate
sensor node mounted on the dome. An optional HC-12 link to a shutter
controller is carried over from the upstream project but was never fitted
here.

This is a single commit from August 2022, published as-is. The design moved
on to an ESP32 in iteration 3 (the ChromeDome repository), so treat this as
a snapshot of a working prototype rather than something maintained.

## Where it came from

The protocol handling, azimuth state machine and command set are adapted
from Juan Menendez Blanco's
[ArduinoDomeController](https://github.com/juanmb/ArduinoDomeController)
(MIT). `serial_command.cpp` and `serial_command.h` are his, unmodified
apart from the file names. The sketch keeps his MaxDome II command table
and EEPROM layout and replaces the position sensing, the display and the
motor control.

## Hardware

Built for an Arduino Nano Every. The pin-8 workaround in `setup()` refers
to that board.

| Function | Pins / bus | Notes |
|---|---|---|
| Drive wheel encoder | D2, D4 | Two hall sensors over a magnet on the gear. D2 is the interrupt. 73 ticks per dome revolution. |
| Home sensor | D3 | Active low. Pulled up. |
| Motor driver | D5 PWM enable, D7 CW, D6 CCW | D8 was enable on schematic v22 / PCB v39 and is jumpered to D5. |
| Azimuth sensor radio | nRF24L01 via NRFLite, CE D9, CSN D10 | This node is radio id 0; the dome sensor is id 1. |
| OLED | SSD1306 128x32 on I2C 0x3C | Shows position, heading, encoder count and calibration state. |
| Push buttons | D11 CW, D12 CCW | Only if `USE_BUTTONS` is defined. Off by default. |
| Shutter link | HC-12 on SoftwareSerial | Only if `HAS_SHUTTER` is defined. Off by default, and the pins are commented out. |
| Host serial | USB, 19200 baud | MaxDome II binary protocol. |

Libraries: SPI, Wire, EEPROM, SoftwareSerial, avr/wdt, NRFLite,
Adafruit_GFX, Adafruit_SSD1306.

## How azimuth is tracked

Two sources feed `current_pos`, which is held in degrees, 0 to 359:

1. **Magnetometer by radio.** The dome sensor node sends a `RadioPacket`
   with its heading, calibration status and battery reading. When a packet
   arrives with calibration above zero, `current_pos` is set to the heading
   plus `compass_offset` (17 degrees by default, the angle between magnetic
   north and the home sensor). If no packet arrives for 750 ms the sensor
   is marked dead.
2. **Drive wheel encoder.** The interrupt on D2 decodes direction from the
   two sensor states on each edge and keeps `encoder_pos` in 0 to 72.
   Position is `encoder_pos` times 360/73. This is the fallback whenever
   the magnetometer is dead or uncalibrated.

Crossing the home sensor zeroes both. During a homing move it also
recomputes `compass_offset` from whatever the magnetometer reports at
home.

Protocol positions are therefore degrees, not encoder ticks as in MaxDome
II. `ticks_per_turn` is pinned at 360 for that reason; the `TICKS_CMD`
handler still writes the value to EEPROM, but the read-back at boot is
commented out.

## Command set

MaxDome II framing: start byte 0x01, length, command id, payload, CRC.
Responses OR the id with 0x80.

| Id | Command | Payload |
|---|---|---|
| 0x03 | Abort | none |
| 0x04 | Home | none |
| 0x05 | Goto | direction byte, 16-bit target in degrees |
| 0x06 | Shutter | sub-command; no-op without `HAS_SHUTTER` |
| 0x07 | Status | returns shutter state, azimuth state, position, home position |
| 0x09 | Set ticks per turn | 16-bit value (stored, not used) |
| 0x0A | Ack | none |
| 0x0B | Set park | park-on-shutter flag, 16-bit park position |
| 0x0C | Battery | returns shutter battery as volts times 100 |

Movement is a four-state machine: idle, moving at full speed, approaching
at 75 percent within 10 degrees of target, homing. Any state times out to
error after 30 seconds. Goto within 5 degrees of the current position is
ignored.

## Building

Open the sketch in the Arduino IDE with the Nano Every board package and
the libraries above installed. The IDE will want the sketch in a folder
named `Observatory-Dome-Controller-Base`; the repository folder is not,
so either rename or let the IDE move it.

I have not compiled this from the repository. There is no Arduino
toolchain on the machine this README was written on, so the notes below
come from reading the code, not from a build.

## Things to know before flashing it

Observations from the source, recorded so nobody has to rediscover them:

- Line 6 reads `#include <Adafruit_SSD1306.h> / Display`. The stray slash
  is extra tokens on a preprocessor line, which GCC warns about but
  accepts.
- `setup()` arms a 2 second watchdog and then calls `delay(5000)` for the
  splash screen. Whether that resets the board depends on how the
  megaAVR core handles `avr/wdt.h`. It evidently ran, so presumably not,
  but I have not checked.
- `callibrateEncoder()` and `goHome()` are never called. Inside
  `getMagHeading()` the statements after `break` are unreachable, and the
  5 second window restarts on every packet, so with a live sensor the
  function would not return. Dead code, but it reads as if it works.
- `callibrateEncoder()` has no return value on its success path.
- In `loop()`, the `else` under the home-sensor check has no braces, so
  only `current_pos = 0` is conditional. The next two lines run every pass
  while the sensor is active, which is harmless but not what the indent
  says.
- `compass_offset` is unsigned and is multiplied by -1 when the home
  position lies counterclockwise of magnetic north. It wraps rather than
  going negative, and `addDegrees` then relies on the wrap to come out
  right.
- The encoder interrupt does four `digitalRead` calls and a float multiply
  per edge. At 73 ticks per revolution that is fine; it would not be on a
  finer encoder.
- The shutter code paths are compiled out and the battery command returns
  the fixed offset of 10.55 V.

## License

GPL-3.0-or-later for this repository; see `LICENSE`. The two
`serial_command` files remain under their original MIT license from
ArduinoDomeController, which is compatible.
