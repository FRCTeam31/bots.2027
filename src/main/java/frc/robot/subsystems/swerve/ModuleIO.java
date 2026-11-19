package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.prime.control.ExtendedPIDConstants;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Hardware layer for one swerve module.
 *
 * <p>Implementations only talk to motors and sensors; all swerve logic lives in
 * {@link Module}. There is one implementation per "world":
 * <ul>
 * <li>{@link ModuleIOTalonFX} - the real robot</li>
 * <li>{@link ModuleIOSim} - physics simulation</li>
 * <li>{@code new ModuleIO() {}} - log replay (inputs come from the log file)</li>
 * </ul>
 */
public interface ModuleIO {

  /** Sensor readings, logged every loop so a match can be replayed in AdvantageKit. */
  @AutoLog
  public static class ModuleIOInputs {
    public double DrivePositionMeters = 0.0;
    public double DriveVelocityMetersPerSec = 0.0;
    public double DriveAppliedVolts = 0.0;
    /** Absolute wheel angle from the CANcoder */
    public Rotation2d SteerAngle = Rotation2d.ZERO;
  }

  /** Reads the latest sensor values into {@code inputs}. */
  default void updateInputs(ModuleIOInputs inputs) {
  }

  /** Runs the drive motor closed-loop at a wheel speed (m/s). */
  default void setDriveVelocity(double metersPerSec) {
  }

  /** Runs the drive motor open-loop at a voltage (used for SysId). */
  default void setDriveVoltage(double volts) {
  }

  /** Turns the wheel to an angle. */
  default void setSteerAngle(Rotation2d angle) {
  }

  /** Stops both motors. */
  default void stop() {
  }

  /** Applies new drive gains (PID + feedforward). */
  default void setDriveGains(ExtendedPIDConstants gains) {
  }

  /** Applies new steering gains. */
  default void setSteerGains(ExtendedPIDConstants gains) {
  }
}
