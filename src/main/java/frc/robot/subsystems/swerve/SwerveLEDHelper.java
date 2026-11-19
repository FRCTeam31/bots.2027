package frc.robot.subsystems.swerve;

import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.led.LEDPattern;

import frc.robot.Container;
import frc.robot.subsystems.leds.LEDPatterns;

/**
 * Resolves LED patterns for strip 1 based on swerve drive state.
 * <ul>
 *   <li><b>Auto-align active</b>: yellow fast blink (locking heading)</li>
 *   <li><b>Autonomous driving</b>: green alternating fast (path following)</li>
 *   <li><b>Normal teleop driving</b>: blue slow blink</li>
 *   <li><b>Disabled</b>: null (no opinion)</li>
 * </ul>
 */
public class SwerveLEDHelper {

    private static final int STRIP_INDEX = 1;
    private static LEDPattern _lastPattern = null;

    /**
     * Resolves the desired LED pattern for the swerve drive's current state.
     * Returns null when disabled (no swerve LED opinion).
     */
    public static LEDPattern resolveDesiredLedPattern(boolean autoAlignEnabled) {
        if (RobotState.isDisabled()) {
            return null;
        }

        if (autoAlignEnabled) {
            return LEDPatterns.YellowFastBlink;
        }

        if (RobotState.isAutonomousEnabled()) {
            return LEDPatterns.GreenAlternatingFast;
        }

        return LEDPatterns.BlueSlowBlink;
    }

    /**
     * Updates LED strip 1 only when the desired pattern changes.
     */
    public static void updateLEDs(boolean autoAlignEnabled) {
        if (Container.LEDs == null)
            return;

        LEDPattern desiredPattern = resolveDesiredLedPattern(autoAlignEnabled);
        if (desiredPattern != null && desiredPattern != _lastPattern) {
            Container.LEDs.setSectionPattern(STRIP_INDEX, desiredPattern);
            _lastPattern = desiredPattern;
        }
    }
}
