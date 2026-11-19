package frc.robot.subsystems.prism;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PrismInputs {
    public boolean Connected = false;
    public long DeviceUptimeMs = 0;
    /** Firmware version reported in the last heartbeat (0 until one arrives) */
    public int FirmwareVersion = 0;
    public int LoopErrorCount = 0;
}
