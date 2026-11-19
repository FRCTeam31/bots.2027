package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.Tunables;

/**
 * One swerve module. Holds the logic shared by real and simulated hardware;
 * the hardware itself is behind {@link ModuleIO}.
 */
public class Module {
  private final String _name;
  private final ModuleIO _io;
  private final ModuleIOInputsAutoLogged _inputs = new ModuleIOInputsAutoLogged();

  // Optimization flips the wheel direction instead of rotating more than 90 degrees
  private final TunableBoolean _optimize;

  public Module(String name, ModuleIO io) {
    _name = name;
    _io = io;
    _optimize = Tunables.addBoolean("Swerve/Modules/" + name + "/Optimize", SwerveMap.OptimizeModulesByDefault);
  }

  /** Reads and logs sensor inputs. Call once per loop, before using any getters. */
  public void periodic() {
    _io.updateInputs(_inputs);
    Logger.processInputs("Swerve/Modules/" + _name, _inputs);
  }

  /** Drives the module toward a target wheel velocity and angle. */
  public void runSetpoint(SwerveModuleVelocity setpoint) {
    var currentAngle = getAngle();

    if (_optimize.get()) {
      setpoint = setpoint.optimize(currentAngle);
    }

    // Slow the wheel while it is still turning toward the target angle, so the
    // robot doesn't get pushed sideways during direction changes
    setpoint = setpoint.cosineScale(currentAngle);

    _io.setDriveVelocity(setpoint.velocity);
    _io.setSteerAngle(setpoint.angle);
    Logger.recordOutput("Swerve/Modules/" + _name + "/Setpoint", setpoint);
  }

  /** Applies a raw drive voltage with the wheel held at an angle (for SysId). */
  public void runCharacterization(double volts, Rotation2d angle) {
    _io.setDriveVoltage(volts);
    _io.setSteerAngle(angle);
  }

  public void stop() {
    _io.stop();
  }

  public void setDriveGains(ExtendedPIDConstants gains) {
    _io.setDriveGains(gains);
  }

  public void setSteerGains(ExtendedPIDConstants gains) {
    _io.setSteerGains(gains);
  }

  public String getName() {
    return _name;
  }

  public Rotation2d getAngle() {
    return _inputs.SteerAngle;
  }

  /** Distance driven and wheel angle, for odometry */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(_inputs.DrivePositionMeters, _inputs.SteerAngle);
  }

  /** Wheel speed and angle */
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(_inputs.DriveVelocityMetersPerSec, _inputs.SteerAngle);
  }

  public double getDriveVoltage() {
    return _inputs.DriveAppliedVolts;
  }
}
