package frc.robot.pneumatics;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.pneumatic.Compressor;
import org.wpilib.hardware.pneumatic.PneumaticHub;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;
import frc.robot.Robot;

public class Pneumatics {
    private PneumaticHub _pcm;
    private Compressor _compressor;

    public Pneumatics() {
        if (Robot.isReal()) {
            _pcm = new PneumaticHub(CANPort.CAN_S0, PneumaticsMap.PCM_CAN_ID);
            _compressor = new Compressor(CANPort.CAN_S0, PneumaticsMap.PCM_CAN_ID, PneumaticsModuleType.CTRE_PCM);

            _compressor.enableDigital();
        }
    }

    public PneumaticHub getPneumaticsControlModule() {
        return _pcm;
    }

    public Integer getPneumaticsControlModuleId() {
        return PneumaticsMap.PCM_CAN_ID;
    }

    public PneumaticsModuleType getPneumaticsControlModuleType() {
        return PneumaticsModuleType.CTRE_PCM;
    }

    public Compressor getCompressor() {
        return _compressor;
    }

    public double getSystemPressure() {
        if (Robot.isReal()) {
            return _pcm.getPressure(PneumaticsMap.PRESSURE_SENSOR_CHANNEL);
        } else {
            return 120.0; // Simulate a constant pressure in simulation
        }
    }
}
