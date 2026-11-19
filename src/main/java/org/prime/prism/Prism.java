package org.prime.prism;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.SerialPort;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Manages serial communication with a single Prism USB LED controller device.
 *
 * <p>Handles strip configuration (with ACK + retry), pixel data streaming,
 * heartbeat health monitoring, and automatic reconnection on disconnect.
 */
public class Prism implements AutoCloseable {

    /**
     * Constants for the Prism USB serial LED controller.
     */
    public static final class PrismMap {

        // ========================= Serial Configuration =========================

        public static final int BAUD_RATE = 2_000_000;
        public static final int DATA_BITS = 8;
        public static final int STOP_BITS = 1;

        // ========================= Frame Framing ================================

        public static final byte SYNC_BYTE_1 = (byte) 0xAA;
        public static final byte SYNC_BYTE_2 = (byte) 0x55;

        // ========================= Commands (Robot -> Device) ===================

        public static final byte CMD_CONFIGURE = 0x01;
        public static final byte CMD_PIXEL_DATA = 0x02;
        public static final byte CMD_PIXEL_DATA_ALL = 0x03;
        public static final byte CMD_HEARTBEAT_REQ = 0x04;

        // ========================= Commands (Device -> Robot) ===================

        public static final byte CMD_CONFIG_ACK = (byte) 0x81;
        public static final byte CMD_HEARTBEAT_RSP = (byte) 0x84;

        // ========================= Status Codes =================================

        public static final byte STATUS_OK = 0x00;
        public static final byte STATUS_ERROR = 0x01;

        // ========================= Strip Configuration ==========================

        public static final int STRIP_COUNT = 4;
        public static final int MAX_PIXELS_PER_STRIP = 144;

        // ========================= Timeouts & Retry =============================

        public static final int CONFIG_ACK_TIMEOUT_MS = 100;
        public static final int HEARTBEAT_TIMEOUT_MS = 500;
        public static final int MAX_RETRIES = 3;
        public static final int RECONNECT_INTERVAL_MS = 1000;
        public static final int HEARTBEAT_INTERVAL_MS = 250;

        // ========================= Frame Overhead ===============================

        /** 2 sync bytes + 1 command + 2 length + 1 checksum = 6 bytes framing overhead */
        public static final int FRAME_OVERHEAD = 6;

        private PrismMap() {
        }
    }

    // ========================= Color Order Enum =============================
    public enum ColorOrder {
        RGB((byte) 0x00),
        GRB((byte) 0x01),
        RGBW((byte) 0x02),
        GRBW((byte) 0x03);

        public final byte value;

        ColorOrder(byte value) {
            this.value = value;
        }
    }

    /**
     * Configuration for a single LED strip on the Prism device.
     */
    public record StripConfig(int pixelCount, ColorOrder colorOrder) {
    }

    private SerialPort _serialPort;
    private final SerialPort.Port _port;
    private volatile boolean _connected;
    private double _lastHeartbeatTime;
    private double _lastHeartbeatRequestTime;
    private volatile long _deviceUptimeMs;
    private volatile int _firmwareVersion;
    private long _lastDeviceUptimeMs;

    // Strip configuration state -- stored for resend on reconnect
    private final int[] _stripPixelCounts = new int[PrismMap.STRIP_COUNT];
    private final ColorOrder[] _stripColorOrders = new ColorOrder[PrismMap.STRIP_COUNT];
    private final boolean[] _stripConfigured = new boolean[PrismMap.STRIP_COUNT];

    private final Alert _disconnectedAlert;

    // Read buffer for responses
    private final byte[] _readBuffer = new byte[64];

    /**
     * Creates a new Prism device and opens the serial port.
     *
     * @param port The USB serial port to use (e.g., SerialPort.Port.USB)
     */
    public Prism(SerialPort.Port port) {
        _port = port;
        _disconnectedAlert = new Alert("prismDisconnected_" + port, "[Prism] Device on " + port + " disconnected.",
                Level.MEDIUM);
        _connected = false;
        _lastHeartbeatTime = 0;
        _lastHeartbeatRequestTime = 0;
        _deviceUptimeMs = 0;
        _lastDeviceUptimeMs = 0;

        tryConnect();
    }

    // ========================= Strip Configuration ==========================

    /**
     * Configures a strip on the device. Sends the configuration frame and
     * waits for acknowledgement with timeout and retry.
     *
     * @param strip      Strip index (0-3)
     * @param pixelCount Number of pixels on this strip
     * @param order      Color order for the strip's LEDs
     * @return true if the device acknowledged the configuration
     */
    public boolean configureStrip(int strip, int pixelCount, ColorOrder order) {
        // Store config for reconnect
        _stripPixelCounts[strip] = pixelCount;
        _stripColorOrders[strip] = order;
        _stripConfigured[strip] = true;

        if (!_connected) {
            return false;
        }

        byte[] frame = Protocol.buildConfigureFrame(strip, pixelCount, order);

        for (int attempt = 0; attempt < PrismMap.MAX_RETRIES; attempt++) {
            try {
                _serialPort.write(frame, frame.length);
                _serialPort.flush();

                // Wait for CONFIG_ACK
                Protocol.PrismResponse response = readResponse(PrismMap.CONFIG_ACK_TIMEOUT_MS);
                if (response instanceof Protocol.ConfigAck ack
                        && ack.stripIndex() == strip
                        && ack.status() == PrismMap.STATUS_OK) {
                    return true;
                }
            } catch (Exception e) {
                DataLogManager.log("[Prism] Config attempt " + (attempt + 1) + " failed: " + e.getMessage());
            }
        }

        DataLogManager
                .log("[Prism] Failed to configure strip " + strip + " after " + PrismMap.MAX_RETRIES + " attempts");
        return false;
    }

    // ========================= Pixel Data ===================================

    /**
     * Sends pixel data for all strips to the device.
     *
     * @param buffers Array of 4 AddressableLEDBuffers (one per strip)
     * @return true if the write succeeded
     */
    public boolean sendPixelData(AddressableLEDBuffer[] buffers) {
        if (!_connected) {
            return false;
        }

        try {
            byte[] frame = Protocol.buildPixelDataAllFrame(buffers);
            _serialPort.write(frame, frame.length);
            return true;
        } catch (Exception e) {
            handleSerialError("sendPixelData", e);
            return false;
        }
    }

    // ========================= Health Monitoring ============================

    /**
     * Sends a heartbeat request if enough time has elapsed since the last one.
     * Should be called periodically from the subsystem's update loop.
     */
    public void periodicHeartbeat() {
        double now = Timer.getTimestamp();

        if (!_connected) {
            // Try to reconnect periodically
            if (now - _lastHeartbeatRequestTime >= PrismMap.RECONNECT_INTERVAL_MS / 1000.0) {
                _lastHeartbeatRequestTime = now;
                tryReconnect();
            }
            return;
        }

        // Send heartbeat request periodically
        if (now - _lastHeartbeatRequestTime >= PrismMap.HEARTBEAT_INTERVAL_MS / 1000.0) {
            _lastHeartbeatRequestTime = now;
            try {
                byte[] frame = Protocol.buildHeartbeatRequest();
                _serialPort.write(frame, frame.length);
            } catch (Exception e) {
                handleSerialError("heartbeat request", e);
                return;
            }
        }

        // Check for heartbeat responses (non-blocking)
        try {
            int available = _serialPort.getBytesReceived();
            if (available > 0) {
                int toRead = Math.min(available, _readBuffer.length);
                byte[] data = _serialPort.read(toRead);
                Protocol.PrismResponse response = Protocol.findAndParseResponse(data, data.length);

                if (response instanceof Protocol.HeartbeatResponse hb) {
                    _lastHeartbeatTime = now;
                    _lastDeviceUptimeMs = _deviceUptimeMs;
                    _deviceUptimeMs = hb.uptimeMs();
                    _firmwareVersion = hb.firmwareVersion();

                    // Detect device reboot (uptime reset)
                    if (_deviceUptimeMs < _lastDeviceUptimeMs && _lastDeviceUptimeMs > 0) {
                        DataLogManager.log("[Prism] Device reboot detected. Resending configuration.");
                        resendAllConfigurations();
                    }
                }
            }
        } catch (Exception e) {
            handleSerialError("heartbeat read", e);
            return;
        }

        // Check for heartbeat timeout
        if (_lastHeartbeatTime > 0 && (now - _lastHeartbeatTime) > PrismMap.HEARTBEAT_TIMEOUT_MS / 1000.0) {
            DataLogManager.log("[Prism] Heartbeat timeout -- marking device disconnected.");
            markDisconnected();
        }
    }

    /**
     * @return true if the device is considered connected (heartbeat received recently)
     */
    public boolean isConnected() {
        return _connected;
    }

    /**
     * @return The last known device uptime in milliseconds
     */
    public long getDeviceUptimeMs() {
        return _deviceUptimeMs;
    }

    /**
     * @return The firmware version from the last heartbeat, or 0 if none received yet
     */
    public int getFirmwareVersion() {
        return _firmwareVersion;
    }

    // ========================= Connection Management ========================

    /**
     * Closes the serial port and releases all resources. The device cannot be
     * used after this is called.
     */
    @Override
    public void close() {
        closePort();
        _disconnectedAlert.close();
    }

    // ========================= Internal =====================================

    private void closePort() {
        if (_serialPort != null) {
            try {
                _serialPort.close();
            } catch (Exception e) {
                // Ignore close errors
            }
            _serialPort = null;
        }
        _connected = false;
        _disconnectedAlert.set(true);
    }

    private void tryConnect() {
        try {
            _serialPort = new SerialPort(PrismMap.BAUD_RATE, _port);
            _serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.FLUSH_ON_ACCESS);
            _serialPort.setTimeout(0.01); // 10ms read timeout
            _connected = true;
            _lastHeartbeatTime = Timer.getTimestamp();
            _disconnectedAlert.set(false);
            DataLogManager.log("[Prism] Connected on port " + _port);
        } catch (Exception e) {
            _connected = false;
            _disconnectedAlert.set(true);
            DriverStationErrors.reportError("[Prism] Failed to open serial port: " + e.getMessage(), false);
        }
    }

    private void tryReconnect() {
        closePort();
        tryConnect();

        if (_connected) {
            DataLogManager.log("[Prism] Reconnected. Resending configuration.");
            resendAllConfigurations();
        }
    }

    private void resendAllConfigurations() {
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            if (_stripConfigured[i]) {
                configureStrip(i, _stripPixelCounts[i], _stripColorOrders[i]);
            }
        }
    }

    private Protocol.PrismResponse readResponse(int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            try {
                int available = _serialPort.getBytesReceived();
                if (available > 0) {
                    int toRead = Math.min(available, _readBuffer.length);
                    byte[] data = _serialPort.read(toRead);
                    Protocol.PrismResponse response = Protocol.findAndParseResponse(data, data.length);
                    if (response != null) {
                        return response;
                    }
                }
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            } catch (Exception e) {
                return null;
            }
        }
        return null;
    }

    private void handleSerialError(String context, Exception e) {
        DataLogManager.log("[Prism] Serial error during " + context + ": " + e.getMessage());
        markDisconnected();
    }

    private void markDisconnected() {
        _connected = false;
        _disconnectedAlert.set(true);
    }
}
