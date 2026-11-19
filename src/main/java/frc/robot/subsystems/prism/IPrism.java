package frc.robot.subsystems.prism;

import org.prime.prism.Prism.ColorOrder;
import org.wpilib.hardware.led.AddressableLEDBuffer;

/**
 * Interface for Prism device IO. Real implementation uses WPILib SerialPort on the
 * robot controller, Sim implementation uses jSerialComm to talk to the real device
 * from the PC. The default methods are no-ops so {@code new IPrism() {}} can be used
 * for log replay.
 */
public interface IPrism {

    /**
     * Updates the inputs for logging.
     */
    default void updateInputs(PrismInputsAutoLogged inputs) {
    }

    /**
     * Configures a strip on the device.
     *
     * @param strip      Strip index (0-3)
     * @param pixelCount Number of pixels
     * @param order      Color order
     * @return true if acknowledged
     */
    default boolean configureStrip(int strip, int pixelCount, ColorOrder order) {
        return false;
    }

    /**
     * Sends pixel data for all strips.
     *
     * @param buffers Array of 4 AddressableLEDBuffers
     * @return true if write succeeded
     */
    default boolean sendPixelData(AddressableLEDBuffer[] buffers) {
        return false;
    }

    /**
     * Periodic heartbeat / health check. Call from update loop.
     */
    default void periodicHeartbeat() {
    }

    /**
     * @return true if device is connected
     */
    default boolean isConnected() {
        return false;
    }

    /**
     * Close the connection and release resources.
     */
    default void close() {
    }
}
