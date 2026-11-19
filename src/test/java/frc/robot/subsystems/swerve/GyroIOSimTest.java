package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

class GyroIOSimTest {
  private double omega;
  private GyroIOSim gyro;
  private GyroIOInputsAutoLogged inputs;

  @BeforeEach
  void setUp() {
    omega = 0;
    gyro = new GyroIOSim(() -> omega);
    inputs = new GyroIOInputsAutoLogged();
  }

  private void runLoops(int loops) {
    for (int i = 0; i < loops; i++) {
      gyro.updateInputs(inputs);
    }
  }

  /** Angle difference that ignores wrapping (e.g. 180 vs -180 is 0). */
  private static double degreesFrom(Rotation2d actual, double expectedDegrees) {
    return actual.minus(Rotation2d.fromDegrees(expectedDegrees)).getDegrees();
  }

  @Test
  void testStartsAtZeroAndConnected() {
    gyro.updateInputs(inputs);

    assertTrue(inputs.Connected);
    assertEquals(0.0, inputs.Yaw.getDegrees(), 1e-9);
  }

  @Test
  void testStationaryHoldsHeading() {
    runLoops(50);
    assertEquals(0.0, inputs.Yaw.getDegrees(), 1e-9);
  }

  @Test
  void testIntegratesCounterClockwiseTurn() {
    omega = Math.PI / 2; // 90 deg/s
    runLoops(50); // 1 second

    assertEquals(0.0, degreesFrom(inputs.Yaw, 90), 0.01);
    assertEquals(Math.PI / 2, inputs.YawVelocityRadPerSec, 1e-9);
  }

  @Test
  void testIntegratesClockwiseTurn() {
    omega = -Math.PI / 2;
    runLoops(50);

    assertEquals(0.0, degreesFrom(inputs.Yaw, -90), 0.01);
  }

  @Test
  void testWrapsPast180() {
    omega = Math.PI; // 180 deg/s
    runLoops(75); // 270 degrees

    assertEquals(0.0, degreesFrom(inputs.Yaw, -90), 0.01);
  }

  @Test
  void testSetYaw() {
    gyro.setYaw(Rotation2d.fromDegrees(180));
    gyro.updateInputs(inputs);

    assertEquals(0.0, degreesFrom(inputs.Yaw, 180), 1e-9);
  }

  @Test
  void testTurningContinuesFromSetYaw() {
    gyro.setYaw(Rotation2d.fromDegrees(45));
    omega = Math.PI / 4; // 45 deg/s
    runLoops(50);

    assertEquals(0.0, degreesFrom(inputs.Yaw, 90), 0.01);
  }

  @Test
  void testAccelerationIsZeroInSim() {
    omega = 1;
    runLoops(10);

    assertEquals(0.0, inputs.AccelX);
    assertEquals(0.0, inputs.AccelY);
    assertEquals(0.0, inputs.AccelZ);
  }
}
