package frc.robot.subsystems.prism;

import org.prime.prism.Prism.ColorOrder;
import org.prime.prism.Prism.StripConfig;
import org.wpilib.hardware.bus.SerialPort;

public class PrismMap {
    public static final int STRIP_COUNT = 4;
    public static final int PIXELS_PER_STRIP = 8; // Firmware supports up to 144
    public static final int UPDATE_RATE_MS = 8; // ~125 fps
    public static final int MAX_LOOP_ERRORS_BEFORE_SHUTDOWN = 3;

    /** Default color order for all strips */
    public static final ColorOrder DEFAULT_COLOR_ORDER = ColorOrder.GRB;

    /** USB serial port the Prism is plugged into on the robot */
    public static final SerialPort.Port REAL_SERIAL_PORT = SerialPort.Port.USB;

    /**
     * Serial port name for simulation (PC serial port to real Prism device).
     * Override with the PRISM_SIM_PORT environment variable, e.g. /dev/ttyACM0 on Linux.
     */
    public static final String SIM_COM_PORT = System.getenv().getOrDefault("PRISM_SIM_PORT", "COM12");
    public static final int SIM_BAUD_RATE = 115200; // Must match firmware Serial.begin() — rate is nominal for USB CDC

    /** PWM port used for the WPILib AddressableLED mirror in simulation */
    public static final int SIM_LED_PWM_PORT = 0;

    /** Default strip configurations */
    public static final StripConfig[] STRIP_CONFIGS = new StripConfig[] {
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
    };
}
