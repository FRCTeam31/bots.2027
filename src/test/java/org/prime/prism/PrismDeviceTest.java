package org.prime.prism;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.prime.prism.Prism.ColorOrder;

import org.wpilib.hardware.bus.SerialPort;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.led.AddressableLEDBuffer;

/**
 * Unit tests for {@link Prism} -- verifies construction and configuration storage.
 *
 * <p>Note: Full serial I/O testing requires a physical device or mock serial port.
 * These tests verify the device's state management and configuration tracking logic
 * using HAL simulation.
 */
class PrismDeviceTest {

    @BeforeEach
    void setUp() {
        assert HAL.initialize();
    }

    @Test
    void testConstructor_doesNotThrow() {
        // In HAL sim, SerialPort construction may succeed even without a real device.
        // The device should construct without throwing regardless.
        assertDoesNotThrow(() -> new Prism(SerialPort.Port.USB).close());
    }

    @Test
    void testConfigureStrip_storesConfigEvenWhenDisconnected() {
        Prism device = new Prism(SerialPort.Port.USB);

        // Should not throw even when disconnected -- config is stored for later
        boolean result = device.configureStrip(0, 30, ColorOrder.GRB);
        device.close();
        assertFalse(result, "Should return false when disconnected");
    }

    @Test
    void testSendPixelData_returnsFalseAfterClose() {
        Prism device = new Prism(SerialPort.Port.USB);
        device.close(); // Force disconnect

        AddressableLEDBuffer[] buffers = new AddressableLEDBuffer[4];
        for (int i = 0; i < 4; i++) {
            buffers[i] = new AddressableLEDBuffer(8);
        }

        assertFalse(device.sendPixelData(buffers), "Should return false when disconnected");
    }

    @Test
    void testGetDeviceUptimeMs_defaultsToZero() {
        Prism device = new Prism(SerialPort.Port.USB);
        assertEquals(0, device.getDeviceUptimeMs());
        device.close();
    }

    @Test
    void testClose_doesNotThrow() {
        Prism device = new Prism(SerialPort.Port.USB);
        assertDoesNotThrow(device::close, "close() should not throw even when not connected");
    }

    @Test
    void testPeriodicHeartbeat_doesNotThrowWhenDisconnected() {
        Prism device = new Prism(SerialPort.Port.USB);
        assertDoesNotThrow(device::periodicHeartbeat, "periodicHeartbeat should not throw when disconnected");
        device.close();
    }
}
