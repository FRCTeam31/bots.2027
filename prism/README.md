# Prism — USB Serial LED Controller Firmware

Prism is a small LED co-processor for the robot. The robot program renders WPILib
`LEDPattern`s into pixel buffers and streams them over USB serial at ~125 fps; Prism
takes those frames and drives up to **4 WS281x strips of up to 144 pixels each**.
Offloading the LED timing to a dedicated microcontroller keeps animations smooth
without tying up the robot controller's own LED hardware.

This folder is a PlatformIO project for the firmware. The robot-side code that talks
to it is documented in
[`src/main/java/frc/robot/subsystems/prism/README.md`](../src/main/java/frc/robot/subsystems/prism/README.md).

## Hardware

| Item | Detail |
|------|--------|
| Board | Seeed Studio XIAO ESP32-S3 |
| Link to robot | Native USB-C (USB CDC serial), plugged into a robot controller USB port |
| LED strips | WS2812 / WS2812B-compatible, GRB, 800 kHz |
| Status LED | 1x SK6812MINI-C on the Prism board, data on **D10** (GPIO9) |

### Pin assignments

| Function | XIAO pin | GPIO |
|----------|----------|------|
| Strip 0 data | D0 | 1 |
| Strip 1 data | D1 | 2 |
| Strip 2 data | D3 | 4 |
| Strip 3 data | D4 | 5 |
| Status LED data | D10 | 9 |

Strip pins are set by `STRIP_PINS` and the status LED by `STATUS_LED_PIN` near the
top of `src/main.cpp`.

### Status LED

| Color | Meaning |
|-------|---------|
| Off | No power, or the firmware isn't running (crashed, in bootloader mode, or not flashed) |
| **Red** | Powered and running, but no robot program is talking to it |
| **Green** | Connected: a valid frame arrived from the robot program in the last 1 s |

"Connected" means a frame passed the checksum, so a cable that enumerates but
carries garbage stays red. The robot sends a heartbeat every 250 ms and pixel data
every ~8 ms, so the LED goes red within about a second if the robot program stops,
crashes, or the USB cable is pulled. The timeout is `HOST_TIMEOUT_MS`, and brightness
is `PRISM_STATUS_LED_BRIGHTNESS` (default 32 of 255; the SK6812MINI-C is bright up close).

## Building and flashing

### Tools

Install **PlatformIO**, either:

- **VS Code extension** (recommended): install "PlatformIO IDE", then open *this
  `prism/` folder* (not the robot project root) as a workspace folder so PlatformIO
  picks up `platformio.ini`.
- **CLI**: `pipx install platformio` (or `pip install platformio`), which provides `pio`.

The first build downloads the ESP32-S3 toolchain, the Arduino-ESP32 core and the
Adafruit NeoPixel library, so it needs internet and takes a few minutes. Later
builds are fast and work offline.

### Build

```bash
cd prism
pio run
```

In VS Code, use the PlatformIO toolbar check-mark (*Build*) or the
`PlatformIO: Build` task.

### Flash (update) a Prism

1. **Free the serial port.** Only one program can hold the port. Unplug Prism from
   the robot and plug it into your laptop, and close anything using it (robot
   simulation with Prism attached, a serial monitor).
2. Flash it:
   ```bash
   cd prism
   pio run -t upload
   ```
   PlatformIO auto-detects the port. If you have several boards attached, pick
   one with `pio run -t upload --upload-port /dev/ttyACM0` (Linux),
   `/dev/cu.usbmodem*` (macOS) or `COMx` (Windows).
3. The board resets after upload. The status LED should turn **red** right away.
4. Optional: run [robot simulation with Prism attached](#bench-testing-without-the-robot)
   and check the LED goes **green** and the strips animate.
5. Plug Prism back into the robot. Once the robot program is running, the LED
   should go **green**, and the `Prism/FirmwareVersion` log entry should show the
   new version.

#### If upload fails ("No serial data received", "port busy", or no port found)

The usual cause is a crashed firmware or USB stack. Force the ESP32-S3 into its ROM
bootloader:

1. Hold the **B** (BOOT) button.
2. While holding it, plug in USB (or tap **R**, RESET).
3. Release BOOT, then run `pio run -t upload` again.
4. Tap **R** after the upload finishes to start the new firmware.

In bootloader mode the status LED stays off. That's expected.

### Versioning

`FIRMWARE_VERSION` in `src/main.cpp` is sent in every heartbeat response and logged
by the robot as `Prism/FirmwareVersion`. **Bump it whenever you change the
firmware** so you can tell from a robot log which build a board is running.

| Version | Change |
|---------|--------|
| `0x0002` | Arduino-framework firmware, 4 strips, heartbeat/config protocol |
| `0x0003` | Onboard SK6812MINI-C status LED on D10 |

### Compile-time options

These are set as `build_flags` in `platformio.ini`:

| Flag | Default | Notes |
|------|---------|-------|
| `PRISM_STRIP_COUNT` | 4 | Must match `Prism.PrismMap.STRIP_COUNT` on the robot side |
| `PRISM_MAX_PIXELS_PER_STRIP` | 144 | Configure requests above this are rejected; sizes the receive buffer |
| `PRISM_STATUS_LED_BRIGHTNESS` | 32 | 0-255 brightness for the onboard status LED |

`sdkconfig.defaults` is left over from the earlier ESP-IDF version of this firmware.
The Arduino framework build doesn't read it.

## Bench testing without the robot

Run the robot program in desktop simulation with Prism plugged into your PC. It
streams the real robot patterns to the hardware. See "Running sim with a real
Prism" in the robot-side README.

For low-level debugging, `src/test/java/frc/robot/subsystems/prism/PrismSerialDebugTest.java`
has a `main()` that opens the port, sends raw heartbeat requests and prints the hex
bytes that come back. Pass the port name as the first argument.

## Serial protocol

All multi-byte integers are little-endian.

```
[0xAA][0x55][cmd:1][payloadLen:2][payload: payloadLen bytes][checksum:1]
```

- `checksum` is the XOR of the payload bytes only (0 for an empty payload).
- The receiver scans for the `AA 55` sync, so stray bytes between frames are dropped.
- Frames with a bad checksum or a `payloadLen` above the maximum are ignored.
- USB CDC ignores the baud rate. The firmware uses 115200 and the robot opens the
  port at 2 Mbaud; both work.

### Robot → Prism

| Cmd | Name | Payload | Response |
|-----|------|---------|----------|
| `0x01` | CONFIGURE | `[strip:1][pixelCount:2][colorOrder:1]` | CONFIG_ACK |
| `0x02` | PIXEL_DATA | `[strip:1][R,G,B × pixelCount]` | none |
| `0x03` | PIXEL_DATA_ALL | for each of the 4 strips: `[pixelCount:2][R,G,B × pixelCount]` | none |
| `0x04` | HEARTBEAT_REQ | empty | HEARTBEAT_RSP |

- **CONFIGURE** (re)creates a strip with the given length and blanks it. It is
  rejected with `STATUS_ERROR` if the strip index is out of range or `pixelCount`
  is 0 or above `PRISM_MAX_PIXELS_PER_STRIP`. `colorOrder` is accepted but
  currently **ignored**: the firmware always drives GRB.
- **PIXEL_DATA_ALL** is what the robot sends every frame. Strips that haven't been
  configured are skipped. If a strip's data has more pixels than its configured
  length, the extra pixels are ignored. All configured strips latch together after
  the frame is applied.
- **PIXEL_DATA** updates one strip. The protocol supports it, but the robot code
  doesn't currently use it.

### Prism → Robot

| Cmd | Name | Payload |
|-----|------|---------|
| `0x81` | CONFIG_ACK | `[strip:1][status:1]`, status `0x00` OK / `0x01` error |
| `0x84` | HEARTBEAT_RSP | `[uptimeMs:4][firmwareVersion:2][status:1]` |

The robot uses `uptimeMs` to spot a Prism reboot (uptime goes backwards) and
re-sends the strip configuration when that happens.

### Keeping both sides in sync

The protocol is implemented twice:

- Firmware: `prism/src/main.cpp`
- Robot: `src/main/java/org/prime/prism/Protocol.java` and `Prism.java` (`PrismMap`)

If you change a command, payload layout or limit, update both, bump
`FIRMWARE_VERSION`, and run the robot tests (`./gradlew test`, which runs
`PrismProtocolTest`).
