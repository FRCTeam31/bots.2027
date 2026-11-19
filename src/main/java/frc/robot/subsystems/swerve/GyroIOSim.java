package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.wpilib.math.geometry.Rotation2d;

/**
 * Simulated gyro: integrates the robot's turn rate (measured from the simulated
 * modules) to produce a heading.
 */
public class GyroIOSim implements GyroIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final DoubleSupplier _omegaRadPerSec;
  private Rotation2d _yaw = Rotation2d.ZERO;

  /** @param omegaRadPerSec the robot's current turn rate, counter-clockwise positive */
  public GyroIOSim(DoubleSupplier omegaRadPerSec) {
    _omegaRadPerSec = omegaRadPerSec;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var omega = _omegaRadPerSec.getAsDouble();
    _yaw = _yaw.plus(Rotation2d.fromRadians(omega * LOOP_PERIOD_SECONDS));

    inputs.Connected = true;
    inputs.Yaw = _yaw;
    inputs.YawVelocityRadPerSec = omega;
    inputs.AccelX = 0;
    inputs.AccelY = 0;
    inputs.AccelZ = 0;
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    _yaw = yaw;
  }
}
