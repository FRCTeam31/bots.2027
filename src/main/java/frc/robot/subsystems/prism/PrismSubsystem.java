package frc.robot.subsystems.prism;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;
import org.prime.prism.Prism.StripConfig;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.system.DataLogManager;
import org.wpilib.units.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color;

import frc.robot.subsystems.leds.ILEDs;

/**
 * Prism LED subsystem using IO abstraction. Works with both {@link PrismReal}
 * on the robot controller and {@link PrismSim} for desktop simulation with real hardware.
 *
 * <p>Renders WPILib {@link LEDPattern}s locally at ~125fps and streams pixel data
 * to the device via the {@link IPrism} interface.
 */
public class PrismSubsystem extends SubsystemBase implements ILEDs {

    private ScheduledExecutorService _updateLoopExecutor;
    private final IPrism _io;
    private final PrismInputsAutoLogged _inputs = new PrismInputsAutoLogged();
    private final AddressableLEDBuffer[] _buffers;
    private final AddressableLEDBufferView[] _sections;
    private final LEDPattern[] _sectionPatterns;
    private volatile int _loopErrorCounter = 0;

    private final LEDPattern _initialPattern = LEDPattern.solid(Color.GHOST_WHITE).breathe(Units.Seconds.of(4));
    private final Alert _loopStoppedAlert = new Alert("prismLoopFailed",
            "[Prism] Update loop has failed too many times. Stopping loop.", Level.HIGH);

    /**
     * Creates the Prism subsystem with the given IO implementation and strip configs.
     *
     * @param io           The IO implementation (Real or Sim)
     * @param stripConfigs Configuration for each strip
     */
    public PrismSubsystem(IPrism io, StripConfig[] stripConfigs) {
        if (stripConfigs.length != PrismMap.STRIP_COUNT) {
            throw new IllegalArgumentException(
                    "Expected " + PrismMap.STRIP_COUNT + " strip configs, got " + stripConfigs.length);
        }

        _io = io;
        _buffers = new AddressableLEDBuffer[PrismMap.STRIP_COUNT];
        _sections = new AddressableLEDBufferView[PrismMap.STRIP_COUNT];
        _sectionPatterns = new LEDPattern[PrismMap.STRIP_COUNT];

        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            var config = stripConfigs[i];

            // Configure device strip
            _io.configureStrip(i, config.pixelCount(), config.colorOrder());

            // Create local buffer for rendering
            _buffers[i] = new AddressableLEDBuffer(config.pixelCount());
            _sections[i] = _buffers[i].createView(0, config.pixelCount() - 1);
            _sectionPatterns[i] = _initialPattern;
        }

        // Apply initial pattern
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            _initialPattern.applyTo(_buffers[i]);
        }

        // Start ~125fps update loop
        startUpdateLoop();
    }

    // ========================= Periodic =====================================

    @Override
    public void periodic() {
        _io.updateInputs(_inputs);
        _inputs.LoopErrorCount = _loopErrorCounter;
        Logger.processInputs("Prism", _inputs);
    }

    // ========================= Update Loop ==================================

    private void updateLoop() {
        if (_loopErrorCounter > PrismMap.MAX_LOOP_ERRORS_BEFORE_SHUTDOWN) {
            stopUpdateLoop();
            return;
        }

        try {
            // Render patterns to local buffers
            for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
                _sectionPatterns[i].applyTo(_sections[i]);
            }

            // Stream pixel data to device
            if (_io.isConnected()) {
                _io.sendPixelData(_buffers);
            }

            // Periodic heartbeat / reconnect
            _io.periodicHeartbeat();
        } catch (Exception e) {
            _loopErrorCounter++;
            DataLogManager.log("[Prism:ERROR] Update loop failed: " + e.getMessage());
            DriverStationErrors.reportError("[Prism:ERROR] Update loop failed: " + e.getMessage(),
                    e.getStackTrace());
        }
    }

    // ========================= Loop Control =================================

    /**
     * Stops the LED update loop.
     */
    public synchronized void stopUpdateLoop() {
        if (_updateLoopExecutor != null) {
            _updateLoopExecutor.shutdown();
            _updateLoopExecutor = null;
        }
        _loopStoppedAlert.set(true);
    }

    /**
     * Starts (or restarts) the LED update loop, resetting the error counter.
     */
    public synchronized void startUpdateLoop() {
        if (_updateLoopExecutor != null) {
            return;
        }

        _loopErrorCounter = 0;
        _loopStoppedAlert.set(false);
        _updateLoopExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            var thread = new Thread(r, "Prism-Update");
            thread.setDaemon(true);
            return thread;
        });
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLoop, 0, PrismMap.UPDATE_RATE_MS, TimeUnit.MILLISECONDS);
    }

    // ========================= ILEDs Implementation =========================

    @Override
    public void setSectionPattern(int section, LEDPattern pattern) {
        _sectionPatterns[section] = pattern;
    }

    @Override
    public void setAllSectionPatterns(LEDPattern pattern) {
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            _sectionPatterns[i] = pattern;
        }
    }

    @Override
    public Command setSectionPatternCommand(int section, LEDPattern pattern) {
        return Commands.runOnce(() -> setSectionPattern(section, pattern)).ignoringDisable(true);
    }

    @Override
    public Command setAllSectionPatternsCommand(LEDPattern pattern) {
        return Commands.runOnce(() -> setAllSectionPatterns(pattern)).ignoringDisable(true);
    }

    // ========================= Accessors ====================================

    public AddressableLEDBuffer getBuffer(int strip) {
        return _buffers[strip];
    }

    public boolean isDeviceConnected() {
        return _io.isConnected();
    }
}
