package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Unit tests for Swerve subsystem - validates subsystem initialization and command creation.
 */
class SwerveTest {
    private Swerve swerve;

    @BeforeEach
    void setUp() {
        assert HAL.initialize();
        swerve = new Swerve();
    }

    @AfterEach
    void tearDown() {
        // 2027 Alerts require unique IDs, so release them before the next test
        swerve.close();
    }

    @Test
    void testConstructor_InitializesInSimulation() {
        swerve.close();
        swerve = assertDoesNotThrow(() -> new Swerve(), "Should initialize without errors");
    }

    @Test
    void testConstructor_SetsSubsystemName() {
        assertEquals("Swerve", swerve.getName());
    }

    @Test
    void testCommandFactories_ReturnValidCommands() {
        assertNotNull(swerve.resetGyroCommand());
        assertNotNull(swerve.stopAllMotorsCommand());
        assertNotNull(swerve.disableAutoAlignCommand());
        assertNotNull(swerve.setAutoAlignSetpointCommand(90));
    }

    @Test
    void testCommandFactories_ReturnNewInstances() {
        var cmd1 = swerve.resetGyroCommand();
        var cmd2 = swerve.resetGyroCommand();
        assertNotSame(cmd1, cmd2, "Each command creation should return a new instance");
    }

    @Test
    void testSetAutoAlignSetpoint_AcceptsEdgeCases() {
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(0));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(360));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(-360));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(720));
    }

    // #region Simulated driving

    /** Runs the subsystem like the robot loop would: periodic, then the drive command. */
    private void runLoops(int loops, ChassisVelocities velocities) {
        for (int i = 0; i < loops; i++) {
            swerve.periodic();
            swerve.driveRobotRelative(velocities);
        }
        swerve.periodic();
    }

    @Test
    void testDrivingForwardMovesPoseForward() {
        runLoops(100, new ChassisVelocities(1.0, 0, 0));

        var pose = swerve.getPose();
        assertTrue(pose.getX() > 1.0, "Robot should have driven forward, x=" + pose.getX());
        assertEquals(0.0, pose.getY(), 0.05, "Robot should not drift sideways");
        assertEquals(1.0, swerve.getMeasuredVelocities().vx, 0.1, "Should reach the commanded speed");
    }

    @Test
    void testTurningUpdatesSimulatedGyro() {
        runLoops(50, new ChassisVelocities(0, 0, Math.PI / 2));

        assertTrue(swerve.getGyroHeading().getDegrees() > 30,
                "Sim gyro should follow module motion, heading=" + swerve.getGyroHeading().getDegrees());
    }

    @Test
    void testSetPoseMovesEstimate() {
        var target = new Pose2d(3, 4, Rotation2d.fromDegrees(90));
        swerve.setPose(target);
        swerve.periodic();

        assertEquals(3, swerve.getPose().getX(), 1e-6);
        assertEquals(4, swerve.getPose().getY(), 1e-6);
        assertEquals(90, swerve.getPose().getRotation().getDegrees(), 1e-6);
    }

    @Test
    void testResetGyroZeroesHeadingAndKeepsPose() {
        runLoops(50, new ChassisVelocities(0.5, 0, 1.0));
        runLoops(25, new ChassisVelocities()); // come to a stop
        var poseBefore = swerve.getPose();

        swerve.resetGyro(); // No alliance in tests, so "forward" is 0 degrees
        swerve.periodic();

        assertEquals(0, swerve.getGyroHeading().getDegrees(), 0.01);
        assertEquals(poseBefore.getX(), swerve.getPose().getX(), 1e-3);
        assertEquals(poseBefore.getRotation().getDegrees(), swerve.getPose().getRotation().getDegrees(), 1e-3);
    }

    @Test
    void testAutoAlignTurnsToSetpoint() {
        swerve.setAutoAlignSetpointCommand(90).initialize();
        assertTrue(swerve.isAutoAlignEnabled());

        runLoops(150, new ChassisVelocities());

        var errorDegrees = swerve.getGyroHeading().minus(Rotation2d.fromDegrees(90)).getDegrees();
        assertEquals(0, errorDegrees, 3, "AutoAlign should face 90 degrees");
    }

    @Test
    void testAutoAlignTakesShortWayAcross180() {
        swerve.setAutoAlignSetpointCommand(170).initialize();
        runLoops(150, new ChassisVelocities());

        // 170 -> -170 is a 20 degree counter-clockwise turn, not 340 clockwise
        swerve.setAutoAlignSetpointCommand(-170).initialize();
        for (int i = 0; i < 50; i++) {
            runLoops(1, new ChassisVelocities());
            var heading = swerve.getGyroHeading().getDegrees();
            assertTrue(Math.abs(heading) > 150, "Should stay near 180, not swing through 0: " + heading);
        }

        var errorDegrees = swerve.getGyroHeading().minus(Rotation2d.fromDegrees(-170)).getDegrees();
        assertEquals(0, errorDegrees, 3, "Should settle at -170");
    }

    @Test
    void testDisableAutoAlignReturnsRotationToDriver() {
        swerve.setAutoAlignSetpointCommand(90).initialize();
        swerve.disableAutoAlignCommand().initialize();

        assertFalse(swerve.isAutoAlignEnabled());
    }

    // #endregion
}
