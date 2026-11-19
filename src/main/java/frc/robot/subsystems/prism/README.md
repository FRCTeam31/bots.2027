# Prism LED subsystem (robot side)

This package drives the **Prism** USB LED controller: a XIAO ESP32-S3 running custom
firmware that drives up to 4 strips of up to 144 LEDs. The robot program does all the
animation work with ordinary WPILib `LEDPattern`s. Prism displays the pixels it is sent.

Firmware build/flash instructions, the status LED, pinout and the full wire protocol
are in [`prism/README.md`](../../../../../../../prism/README.md) at the repo root.

## How it fits together

```
 Commands / helpers                     PrismSubsystem ("Prism-Update" thread, every 8 ms)
 ───────────────────                    ──────────────────────────────────────────────────
 MatchLEDHelper  ──setSectionPattern──▶  _sectionPatterns[0..3]
 SwerveLEDHelper ──setSectionPattern──▶        │ applyTo()
 Robot / commands ─set*PatternCommand─▶        ▼
                                         AddressableLEDBuffer[4]  (one per strip)
                                               │ IPrism.sendPixelData()
                                               ▼
                         ┌─────────────── IPrism ────────────────┐
                         │ PrismReal  → org.prime.prism.Prism     │ robot: WPILib SerialPort (USB)
                         │ PrismSim   → jSerialComm, own thread   │ desktop sim: PC serial port
                         │ new IPrism() {}  (all no-ops)          │ AdvantageKit log replay
                         └────────────────────────────────────────┘
                                               │ USB CDC serial frames
                                               ▼
                                        Prism firmware → 4 LED strips
```

| Class | Role |
|-------|------|
| `PrismSubsystem` | Implements `ILEDs`. Owns one buffer and pattern per strip, runs the 125 fps render/stream loop, logs `PrismInputs`. |
| `IPrism` | IO layer (AdvantageKit style). Its default methods are no-ops, used for replay. |
| `PrismReal` | Robot IO. Thin wrapper around `org.prime.prism.Prism`. |
| `PrismSim` | Desktop-sim IO. Talks to a real Prism plugged into your PC and mirrors the pixels into the sim GUI. |
| `PrismMap` | Robot config: strip lengths, update rate, serial ports. |
| `org.prime.prism.Prism` | Serial driver: connect, configure with ACK and retry, stream pixels, heartbeat, reconnect. Holds the protocol constants (`Prism.PrismMap`). |
| `org.prime.prism.Protocol` | Builds and parses frames. Pure functions, unit tested in `PrismProtocolTest`. |

`Container` picks the IO from `Robot.getMode()`:

```java
LEDs = new PrismSubsystem(
    switch (Robot.getMode()) {
      case REAL -> new PrismReal(PrismMap.REAL_SERIAL_PORT);
      case SIM -> new PrismSim(PrismMap.SIM_COM_PORT, PrismMap.SIM_BAUD_RATE);
      case REPLAY -> new IPrism() {};
    },
    PrismMap.STRIP_CONFIGS);
```

`Container.LEDs` is typed as `ILEDs`, the section-based LED API that the rest of
the robot code uses.

## Setting patterns

Each Prism strip is one *section* in the `ILEDs` API:

```java
Container.LEDs.setSectionPattern(1, LEDPatterns.YellowFastBlink);       // immediate
Container.LEDs.setAllSectionPatterns(LEDPattern.solid(Color.RED));
Container.LEDs.setSectionPatternCommand(3, LEDPatterns.RedQuickFlash);  // as a Command (runs while disabled)
```

Patterns are rendered on every update, so time-based patterns (`blink`, `breathe`,
`scrollAtRelativeVelocity`, …) animate on their own. You only call
`setSectionPattern` when you want a *different* pattern. Reusable patterns live in
`leds/LEDPatterns.java`, and building blocks are in `org.prime.util.LEDPatternUtil`.

Current strip ownership:

| Strip | Owner | Shows |
|-------|-------|-------|
| 0 | none | Unassigned; boot pattern until something sets it |
| 1 | `swerve/SwerveLEDHelper` (called from `Swerve.periodic()`) | Auto-align, autonomous, teleop driving |
| 2 | none | Unassigned |
| 3 | `leds/MatchLEDHelper` (called from `Robot.robotPeriodic()`) | E-stop, disabled (alliance breathe), auto, teleop, endgame |

Every strip starts on a ghost-white breathe. `Robot` also sets **all** strips to a
green flash followed by an alliance-color breathe when the robot is disabled after
teleop.

To give a subsystem its own strip, follow the helper pattern: a static
`resolveDesiredLedPattern(...)` that returns a pattern for the current state, plus
an `updateLEDs(...)` that calls `setSectionPattern` only when the result changes.
Return **the same `LEDPattern` instance** for the same state (a constant or a cached
field). If you build a new pattern on every call, the change check never matches
and the pattern gets replaced every loop.

## Timing and threading

- The render/stream loop runs on a daemon thread named `Prism-Update` every
  `PrismMap.UPDATE_RATE_MS` (8 ms, about 125 fps), separate from the 20 ms robot loop.
  Each tick it renders every strip's pattern into its buffer, sends one
  `PIXEL_DATA_ALL` frame if connected, then runs heartbeat/reconnect handling.
- `setSectionPattern` just swaps an array element, so it's safe to call from the main
  robot thread while the update thread renders.
- `PrismSubsystem.periodic()` (main thread) only copies IO state into `PrismInputs`
  and logs it.
- If the loop throws more than `MAX_LOOP_ERRORS_BEFORE_SHUTDOWN` (3) times, it stops
  and raises the *"Update loop has failed too many times"* alert.
  `startUpdateLoop()` restarts it.
- Bandwidth: 4 × 144 pixels × 3 bytes is about 1.7 KB per frame, or about 220 KB/s at
  125 fps. USB CDC (full-speed USB) handles that easily. The baud rate setting doesn't apply.

## Connection lifecycle (`org.prime.prism.Prism`)

1. **Open**: the constructor opens `SerialPort.Port.USB`. If the port opens, the
   driver counts as connected. If it doesn't open (no Prism plugged in), it isn't,
   and the *"[Prism] Device on USB disconnected"* alert is raised.
2. **Configure**: `PrismSubsystem` sends CONFIGURE for each strip in
   `PrismMap.STRIP_CONFIGS`, waiting up to 100 ms for an ACK with 3 tries per
   strip. The configs are stored so they can be re-sent later.
3. **Stream**: pixel frames go out every tick while connected.
4. **Heartbeat**: a HEARTBEAT_REQ goes out every 250 ms. If no response arrives
   within 500 ms, the driver marks itself disconnected.
5. **Reconnect**: while disconnected, it closes and reopens the port every 1 s and
   re-sends all strip configs on success.
6. **Reboot detection**: if the uptime in a heartbeat goes backwards, Prism has
   rebooted (brown-out, reset button), so all strip configs are re-sent.

The firmware's own status LED shows the other side of the link: **green** while it
receives valid frames, **red** when it has gone 1 s without any.

## Configuration (`PrismMap`)

| Constant | Default | Notes |
|----------|---------|-------|
| `PIXELS_PER_STRIP` | 8 | Set to the real strip lengths. Max 144 (the firmware rejects larger). |
| `STRIP_CONFIGS` | 4 × (`PIXELS_PER_STRIP`, GRB) | One entry per strip. Lengths can differ per strip. Must have exactly 4 entries. |
| `UPDATE_RATE_MS` | 8 | Render/stream period |
| `REAL_SERIAL_PORT` | `SerialPort.Port.USB` | Port on the robot controller |
| `SIM_COM_PORT` | `COM12` | PC serial port for sim; override with the `PRISM_SIM_PORT` env var |
| `SIM_LED_PWM_PORT` | 0 | Channel for the sim-GUI LED mirror |

Protocol-level constants (timeouts, retry counts, command bytes) are in
`org.prime.prism.Prism.PrismMap`. They have to match the firmware.

The color order in `StripConfig` is sent to the device, but the firmware currently
always drives GRB. Supporting other orders needs a firmware change.

## Running sim with a real Prism

1. Plug Prism into your PC over USB and find its port: `/dev/ttyACM0` (Linux),
   `/dev/cu.usbmodem…` (macOS) or `COMx` in Device Manager (Windows).
2. Point sim at it: `export PRISM_SIM_PORT=/dev/ttyACM0` (or set it in your IDE
   run config), then run the simulator (`./gradlew run`).
3. `PrismSim` opens the port on its own `PrismSim-IO` thread, waits 2 s and discards
   any boot output, then sends the strip configs once the first heartbeat comes
   back. Prism's status LED turns green.
4. While connected, the pixels are also mirrored into the sim GUI's Addressable
   LED view, with all strips joined end to end.

If no Prism is attached, sim logs `[PrismSim] Serial thread error …` once and keeps
running without LEDs. That's expected. The sim heartbeat timeout is 3× the real
one, because desktop USB CDC is less consistent.

## Logged data

Logged as AdvantageKit inputs under `Prism/`:

| Key | Meaning |
|-----|---------|
| `Connected` | Driver considers the link up |
| `DeviceUptimeMs` | Prism uptime from the last heartbeat |
| `FirmwareVersion` | Firmware version from the last heartbeat (0 until the first one). Use it to confirm a flash took. |
| `LoopErrorCount` | Exceptions thrown by the update loop |

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Prism status LED **off** | No power or firmware not running. Reflash (see `prism/README.md`). |
| Status LED **red**, robot is on | The robot isn't sending valid frames. Check the USB cable and port, look for the `Prism disconnected` alert, and check `Prism/Connected`. |
| Status LED **green**, strips dark | Strips not configured, or wrong length. Look for `Failed to configure strip` in the log and check `PIXELS_PER_STRIP` is between 1 and 144. Check strip power and the data pin. |
| Wrong colors (red and green swapped) | The strips aren't GRB. The firmware is hard-coded to GRB. |
| Only part of a strip lights | `PIXELS_PER_STRIP` / `STRIP_CONFIGS` is shorter than the physical strip. |
| `Update loop has failed too many times` alert | A pattern threw an exception. See the logged stack trace. |
| Sim can't connect | Wrong `PRISM_SIM_PORT`, or another program (serial monitor, `pio`) has the port open. |
