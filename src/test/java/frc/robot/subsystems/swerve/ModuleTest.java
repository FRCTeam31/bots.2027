package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Tests the shared module logic using a fake {@link ModuleIO} that records
 * what the module asks the hardware to do.
 */
class ModuleTest {

  /** Fake hardware: reports a fixed wheel angle and remembers commands. */
  private static class FakeModuleIO implements ModuleIO {
    Rotation2d currentAngle = Rotation2d.ZERO;
    double commandedVelocity = Double.NaN;
    double commandedVolts = Double.NaN;
    Rotation2d commandedAngle = null;
    boolean stopped = false;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.SteerAngle = currentAngle;
      inputs.DrivePositionMeters = 1.5;
      inputs.DriveVelocityMetersPerSec = 2.0;
    }

    @Override
    public void setDriveVelocity(double metersPerSec) {
      commandedVelocity = metersPerSec;
    }

    @Override
    public void setDriveVoltage(double volts) {
      commandedVolts = volts;
    }

    @Override
    public void setSteerAngle(Rotation2d angle) {
      commandedAngle = angle;
    }

    @Override
    public void stop() {
      stopped = true;
    }
  }

  private FakeModuleIO io;
  private Module module;

  @BeforeEach
  void setUp() {
    io = new FakeModuleIO();
    module = new Module("Test", io);
    module.periodic();
  }

  @Test
  void testSetpointAlignedWithWheelPassesThrough() {
    module.runSetpoint(new SwerveModuleVelocity(2.0, Rotation2d.ZERO));

    assertEquals(2.0, io.commandedVelocity, 1e-9);
    assertEquals(0.0, io.commandedAngle.getDegrees(), 1e-9);
  }

  @Test
  void testCosineScalingSlowsWheelWhileTurning() {
    // Wheel points at 0 deg but should point at 60 deg: drive at cos(60) = half speed
    module.runSetpoint(new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(60)));

    assertEquals(1.0, io.commandedVelocity, 1e-9);
    assertEquals(60.0, io.commandedAngle.getDegrees(), 1e-9);
  }

  @Test
  void testPerpendicularSetpointDoesNotDrive() {
    module.runSetpoint(new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(90)));

    assertEquals(0.0, io.commandedVelocity, 1e-9);
  }

  @Test
  void testCharacterizationUsesVoltageAndHoldsAngle() {
    module.runCharacterization(4.0, Rotation2d.ZERO);

    assertEquals(4.0, io.commandedVolts, 1e-9);
    assertEquals(0.0, io.commandedAngle.getDegrees(), 1e-9);
  }

  @Test
  void testGettersReflectInputs() {
    io.currentAngle = Rotation2d.fromDegrees(30);
    module.periodic();

    assertEquals(1.5, module.getPosition().distance, 1e-9);
    assertEquals(2.0, module.getVelocity().velocity, 1e-9);
    assertEquals(30.0, module.getAngle().getDegrees(), 1e-9);
  }

  @Test
  void testStop() {
    module.stop();
    assertTrue(io.stopped);
  }
}
