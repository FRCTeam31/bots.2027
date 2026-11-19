package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Hardware layer for the drivetrain gyro.
 * Implementations: {@link GyroIOPigeon2} (real), {@link GyroIOSim} (simulation),
 * and {@code new GyroIO() {}} (log replay).
 */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean Connected = false;
    /** Robot heading, counter-clockwise positive */
    public Rotation2d Yaw = Rotation2d.ZERO;
    public double YawVelocityRadPerSec = 0.0;
    /** Accelerations in g, used for impact rumble */
    public double AccelX = 0.0;
    public double AccelY = 0.0;
    public double AccelZ = 0.0;
  }

  default void updateInputs(GyroIOInputs inputs) {
  }

  /** Re-zeroes the gyro so it reads {@code yaw}. */
  default void setYaw(Rotation2d yaw) {
  }
}
