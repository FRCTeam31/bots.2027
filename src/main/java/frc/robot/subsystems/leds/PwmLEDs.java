package frc.robot.subsystems.leds;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import org.wpilib.units.Units;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.util.Alert;
import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class PwmLEDs extends SubsystemBase {
    private final ScheduledExecutorService _updateLoopExecutor = Executors.newScheduledThreadPool(1);
    private AddressableLED _led;
    private AddressableLEDBuffer _ledBuffer;
    private AddressableLEDBufferView[] _sections = new AddressableLEDBufferView[LEDMap.SectionCount];
    private LEDPattern[] _sectionPatterns = new LEDPattern[LEDMap.SectionCount];
    public byte _loopErrorCounter = 0;

    private LEDPattern _initialPattern = LEDPattern.solid(Color.GHOST_WHITE).breathe(Units.Seconds.of(4));
    private Alert _loopStoppedAlert = new Alert("ledLoopFailed",
            "Update loop has failed 3 times. Stopping loop.",
            Level.HIGH);

    public PwmLEDs() {
        // Initialize the LED strip and buffer
        _ledBuffer = new AddressableLEDBuffer(LEDMap.TotalPixels);
        for (var i = 0; i < LEDMap.SectionCount; i++) {
            var startingIndex = i * LEDMap.PixelsPerSection;
            var endingIndex = startingIndex + LEDMap.PixelsPerSection - 1; // minus 1 here because the starting and
                                                                           // ending indexes are inclusive
            _sections[i] = _ledBuffer.createView(startingIndex, endingIndex);
            _sectionPatterns[i] = _initialPattern;
        }

        _led = new AddressableLED(LEDMap.PwmPort);
        _led.setLength(_ledBuffer.getLength());

        // LED update loop running at 125fps
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLedStrip, 0, 8, java.util.concurrent.TimeUnit.MILLISECONDS);

        // Apply a default pattern to the LED strip
        _initialPattern.applyTo(_ledBuffer);

        // Setup the warning for when the loop stops
        _loopStoppedAlert.set(false);
    }

    /**
     * Stops the update loop for the LEDs.
     */
    public void stopUpdateLoop() {
        _updateLoopExecutor.shutdown();
        _loopStoppedAlert.set(true);
    }

    /**
     * Starts the update loop for the LEDs.
     */
    public void startUpdateLoop() {
        _loopErrorCounter = 0;
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLedStrip, 0, 8, java.util.concurrent.TimeUnit.MILLISECONDS);
    }

    /**
     * Updates the LED strip with the current patterns.
     */
    private void updateLedStrip() {
        // If we've failed too many times, stop the loop and alert the user
        if (_loopErrorCounter > LEDMap.MaxLoopErrorsBeforeShutdown) {
            _loopStoppedAlert.set(true);
            stopUpdateLoop();
        }

        _loopStoppedAlert.set(false);

        try {
            for (var i = 0; i < LEDMap.SectionCount; i++) {
                _sectionPatterns[i].applyTo(_sections[i]);
            }

            // Update the LED strip with the new data
            _led.setData(_ledBuffer);
        } catch (Exception e) {
            // If we fail to update the LEDs, report the error and increment the error
            // counter
            _loopErrorCounter++;
            DataLogManager.log("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage());
            DriverStationErrors.reportError("[LEDs:ERROR] Failed to update LEDs: " + e.getMessage(),
                    e.getStackTrace());
        }
    }

    public void setSectionPattern(int section, LEDPattern pattern) {
        _sectionPatterns[section] = pattern;
    }

    public void setAllSectionPatterns(LEDPattern pattern) {
        for (var i = 0; i < LEDMap.SectionCount; i++) {
            _sectionPatterns[i] = pattern;
        }
    }

    public Command setSectionPatternCommand(int section, LEDPattern pattern) {
        return Commands.runOnce(() -> setSectionPattern(section, pattern)).ignoringDisable(true);
    }

    public Command setAllSectionPatternsCommand(LEDPattern pattern) {
        return Commands.runOnce(() -> setAllSectionPatterns(pattern)).ignoringDisable(true);
    }
}
