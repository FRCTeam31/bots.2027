package frc.robot.subsystems.swerve;

import org.prime.control.ExtendedPIDConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

/**
 * Simulated swerve module. The drive wheel is a physics model; steering turns
 * at the real module's Motion Magic cruise speed.
 */
public class ModuleIOSim implements ModuleIO {
  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1); // matches DrivePID kV (~0.12 V/motor rps x 5.9)
  private static final double DRIVE_MOMENT_OF_INERTIA = 0.001; // kg*m^2
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double STEER_MAX_RADIANS_PER_LOOP = 15 * 2 * Math.PI * LOOP_PERIOD_SECONDS; // 15 rot/s

  // 2027: LinearSystemId.createDCMotorSystem() was replaced by Models. The
  // single-jointed-arm model is the same [position, velocity] motor + inertia plant.
  private final DCMotorSim _driveSim = new DCMotorSim(
      Models.singleJointedArmFromPhysicalConstants(DRIVE_MOTOR, DRIVE_MOMENT_OF_INERTIA, SwerveMap.DriveGearRatio),
      DRIVE_MOTOR);

  // Drive control runs in wheel rotations/sec, like the real TalonFX
  private PIDController _drivePid;
  private SimpleMotorFeedforward _driveFeedforward;
  private boolean _driveClosedLoop = false;
  private double _driveSetpointWheelRps = 0.0;
  private double _driveAppliedVolts = 0.0;

  private Rotation2d _steerAngle = Rotation2d.ZERO;
  private Rotation2d _steerTarget = Rotation2d.ZERO;

  public ModuleIOSim() {
    setDriveGains(SwerveMap.DrivePID);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (_driveClosedLoop) {
      var currentWheelRps = _driveSim.getAngularVelocity() / (2 * Math.PI);
      var volts = _driveFeedforward.calculate(_driveSetpointWheelRps)
          + _drivePid.calculate(currentWheelRps, _driveSetpointWheelRps);
      _driveAppliedVolts = Math.clamp(volts, -12.0, 12.0);
    }

    // Turn toward the target the short way, limited to the steering motor's speed
    var steerError = _steerTarget.minus(_steerAngle).getRadians();
    var steerStep = Math.clamp(steerError, -STEER_MAX_RADIANS_PER_LOOP, STEER_MAX_RADIANS_PER_LOOP);
    _steerAngle = _steerAngle.plus(Rotation2d.fromRadians(steerStep));

    _driveSim.setInputVoltage(_driveAppliedVolts);
    _driveSim.update(LOOP_PERIOD_SECONDS);

    // DCMotorSim reports the wheel (after gearing) in radians
    var metersPerRadian = SwerveMap.DriveWheelCircumferenceMeters / (2 * Math.PI);
    inputs.DrivePositionMeters = _driveSim.getAngularPosition() * metersPerRadian;
    inputs.DriveVelocityMetersPerSec = _driveSim.getAngularVelocity() * metersPerRadian;
    inputs.DriveAppliedVolts = _driveAppliedVolts;
    inputs.SteerAngle = _steerAngle;
  }

  @Override
  public void setDriveVelocity(double metersPerSec) {
    _driveClosedLoop = true;
    _driveSetpointWheelRps = metersPerSec / SwerveMap.DriveWheelCircumferenceMeters;
  }

  @Override
  public void setDriveVoltage(double volts) {
    _driveClosedLoop = false;
    _driveAppliedVolts = volts;
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    _steerTarget = angle;
  }

  @Override
  public void stop() {
    setDriveVoltage(0);
    _driveSim.setAngularVelocity(0);
  }

  @Override
  public void setDriveGains(ExtendedPIDConstants gains) {
    _drivePid = gains.createPIDController(LOOP_PERIOD_SECONDS);
    _driveFeedforward = gains.createSimpleMotorFeedForward();
  }
}
