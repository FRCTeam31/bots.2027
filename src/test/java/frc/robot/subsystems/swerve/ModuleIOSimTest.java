package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;

class ModuleIOSimTest {
  private ModuleIOSim io;
  private ModuleIOInputsAutoLogged inputs;

  @BeforeEach
  void setUp() {
    assert HAL.initialize();
    io = new ModuleIOSim();
    inputs = new ModuleIOInputsAutoLogged();
  }

  private void runLoops(int loops) {
    for (int i = 0; i < loops; i++) {
      io.updateInputs(inputs);
    }
  }

  @Test
  void testStartsAtRest() {
    io.updateInputs(inputs);

    assertEquals(0.0, inputs.DrivePositionMeters, 1e-9);
    assertEquals(0.0, inputs.DriveVelocityMetersPerSec, 1e-9);
    assertEquals(0.0, inputs.SteerAngle.getDegrees(), 1e-9);
  }

  @Test
  void testVelocityControlReachesSetpoint() {
    io.setDriveVelocity(2.0);
    runLoops(100);

    assertEquals(2.0, inputs.DriveVelocityMetersPerSec, 0.1);
  }

  @Test
  void testNegativeVelocityControl() {
    io.setDriveVelocity(-1.5);
    runLoops(100);

    assertEquals(-1.5, inputs.DriveVelocityMetersPerSec, 0.1);
  }

  @Test
  void testPositionAccumulatesWhileDriving() {
    io.setDriveVelocity(2.0);
    runLoops(100); // 2 seconds

    assertTrue(inputs.DrivePositionMeters > 3.0, "Drove " + inputs.DrivePositionMeters + "m");
  }

  @Test
  void testVoltageModeIsOpenLoop() {
    io.setDriveVoltage(6.0);
    runLoops(10);

    assertEquals(6.0, inputs.DriveAppliedVolts, 1e-9);
    assertTrue(inputs.DriveVelocityMetersPerSec > 0);
  }

  @Test
  void testSteeringReachesTarget() {
    io.setSteerAngle(Rotation2d.fromDegrees(45));
    io.updateInputs(inputs);

    assertEquals(45.0, inputs.SteerAngle.getDegrees(), 1e-9, "A small turn completes in one loop");
  }

  @Test
  void testSteeringIsRateLimitedAndTakesShortWay() {
    io.setSteerAngle(Rotation2d.fromDegrees(-170));
    io.updateInputs(inputs);

    // 15 rot/s steering covers 108 degrees per 20ms loop, clockwise is shorter here
    assertEquals(-108.0, inputs.SteerAngle.getDegrees(), 1e-6);

    io.updateInputs(inputs);
    assertEquals(-170.0, inputs.SteerAngle.getDegrees(), 1e-6);
  }

  @Test
  void testStopHaltsWheel() {
    io.setDriveVelocity(2.0);
    runLoops(50);
    io.stop();
    io.updateInputs(inputs);

    assertEquals(0.0, inputs.DriveAppliedVolts, 1e-9);
    assertEquals(0.0, inputs.DriveVelocityMetersPerSec, 0.05);
  }
}
