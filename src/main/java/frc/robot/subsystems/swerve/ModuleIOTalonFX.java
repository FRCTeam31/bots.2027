package frc.robot.subsystems.swerve;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.function.Supplier;

import org.prime.control.ExtendedPIDConstants;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Real swerve module: Kraken/Falcon TalonFX drive + TalonFX steering with a
 * remote CANcoder.
 *
 * <p>Units on the motor controllers are wheel rotations (drive) and wheel
 * rotations (steering), because the gear ratios are configured on the devices.
 */
public class ModuleIOTalonFX implements ModuleIO {
  // Status signals only need to be fresh for each 50Hz loop. Keep the rate modest:
  // all drive devices share one Systemcore CAN bus with no CANivore.
  private static final double SIGNAL_UPDATE_HZ = 100;
  private static final int CONFIG_ATTEMPTS = 5;

  private final String _name;
  private final TalonFX _driveMotor;
  private final TalonFX _steerMotor;
  private final CANcoder _encoder;

  private final StatusSignal<Angle> _drivePosition; // wheel rotations
  private final StatusSignal<AngularVelocity> _driveVelocity; // wheel rotations/sec
  private final StatusSignal<Voltage> _driveVoltage;
  private final StatusSignal<Angle> _steerAbsolutePosition; // wheel rotations, from CANcoder

  private final VelocityVoltage _driveVelocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final VoltageOut _driveVoltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage _steerRequest = new MotionMagicVoltage(0).withSlot(0);

  public ModuleIOTalonFX(ModuleConfig config, CANBus bus) {
    _name = config.name();
    _encoder = new CANcoder(config.encoderId(), bus);
    _steerMotor = new TalonFX(config.steerMotorId(), bus);
    _driveMotor = new TalonFX(config.driveMotorId(), bus);

    // The CANcoder is configured first because the steering motor uses it as its sensor
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig.MagnetSensor.MagnetOffset = -config.encoderOffsetRotations();
    applyConfig("CANcoder", () -> _encoder.getConfigurator().apply(encoderConfig));

    var steerConfig = createSteerConfig(config);
    applyConfig("steer motor", () -> _steerMotor.getConfigurator().apply(steerConfig));

    var driveConfig = createDriveConfig(config);
    applyConfig("drive motor", () -> _driveMotor.getConfigurator().apply(driveConfig));

    _drivePosition = _driveMotor.getPosition(false);
    _driveVelocity = _driveMotor.getVelocity(false);
    _driveVoltage = _driveMotor.getMotorVoltage(false);
    _steerAbsolutePosition = _encoder.getPosition(false);

    BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
        _drivePosition, _driveVelocity, _driveVoltage, _steerAbsolutePosition);
    ParentDevice.optimizeBusUtilizationForAll(_driveMotor, _steerMotor, _encoder);

    _encoder.clearStickyFaults();
    _steerMotor.clearStickyFaults();
    _driveMotor.clearStickyFaults();
  }

  private static TalonFXConfiguration createSteerConfig(ModuleConfig module) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = module.steerInverted()
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;

    // Use the CANcoder as the steering sensor. FusedCANcoder would be better but needs Phoenix Pro.
    config.Feedback.FeedbackRemoteSensorID = module.encoderId();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = SwerveMap.SteeringGearRatio;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0 = toSlot0(SwerveMap.SteeringPID);
    config.MotionMagic.MotionMagicCruiseVelocity = 15; // rotations/sec
    config.MotionMagic.MotionMagicAcceleration = 100; // rotations/sec^2
    config.MotionMagic.MotionMagicJerk = 1000; // rotations/sec^3

    // The wheel angle wraps around, so 0.9 -> 0.1 rotations is a short 0.2 turn
    config.ClosedLoopGeneral.ContinuousWrap = true;
    return config;
  }

  private static TalonFXConfiguration createDriveConfig(ModuleConfig module) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = module.driveInverted()
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;

    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    // Report position/velocity in wheel rotations instead of motor rotations
    config.Feedback.SensorToMechanismRatio = SwerveMap.DriveGearRatio;

    config.Slot0 = toSlot0(SwerveMap.DrivePID);
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveMap.DriveMotorRampRate;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    return config;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveVoltage, _steerAbsolutePosition);

    var circumference = SwerveMap.DriveWheelCircumferenceMeters;
    inputs.DrivePositionMeters = _drivePosition.getValue().in(Rotations) * circumference;
    inputs.DriveVelocityMetersPerSec = _driveVelocity.getValue().in(RotationsPerSecond) * circumference;
    inputs.DriveAppliedVolts = _driveVoltage.getValue().in(Volts);
    inputs.SteerAngle = Rotation2d.fromRotations(_steerAbsolutePosition.getValue().in(Rotations));
  }

  @Override
  public void setDriveVelocity(double metersPerSec) {
    var wheelRps = metersPerSec / SwerveMap.DriveWheelCircumferenceMeters;
    _driveMotor.setControl(_driveVelocityRequest.withVelocity(wheelRps));
  }

  @Override
  public void setDriveVoltage(double volts) {
    _driveMotor.setControl(_driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    _steerMotor.setControl(_steerRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void stop() {
    _driveMotor.stopMotor();
    _steerMotor.stopMotor();
  }

  @Override
  public void setDriveGains(ExtendedPIDConstants gains) {
    _driveMotor.getConfigurator().apply(toSlot0(gains));
  }

  @Override
  public void setSteerGains(ExtendedPIDConstants gains) {
    _steerMotor.getConfigurator().apply(toSlot0(gains));
  }

  private static Slot0Configs toSlot0(ExtendedPIDConstants gains) {
    var slot0 = new Slot0Configs();
    slot0.kP = gains.kP;
    slot0.kI = gains.kI;
    slot0.kD = gains.kD;
    slot0.kS = gains.kS;
    slot0.kV = gains.kV;
    slot0.kA = gains.kA;
    return slot0;
  }

  /** Applies a device config, retrying a few times in case the device is still booting. */
  private void applyConfig(String device, Supplier<StatusCode> apply) {
    var status = StatusCode.StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CONFIG_ATTEMPTS && !status.isOK(); attempt++) {
      status = apply.get();
    }
    if (!status.isOK()) {
      DriverStationErrors.reportWarning("[SWERVE] " + _name + " " + device + " config failed: " + status, false);
    }
  }
}
