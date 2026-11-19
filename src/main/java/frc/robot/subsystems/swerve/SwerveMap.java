package frc.robot.subsystems.swerve;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import org.prime.control.ExtendedPIDConstants;

/**
 * Swerve drivetrain constants: geometry, gains, and CAN IDs.
 */
public class SwerveMap {
        public class Chassis {
                public static final double TrackWidthMeters = Units.inchesToMeters(24.75);
                public static final double WheelBaseMeters = Units.inchesToMeters(24.75);
                public static final double MaxSpeedMetersPerSecond = 5.4;
                public static final double MaxAngularSpeedRadians = Math.PI * 2 * 0.75;
                // AutoAlign turn acceleration limit (rad/s^2): reaches max turn speed in ~0.25s. Tune on robot.
                public static final double MaxAngularAccelerationRadians = MaxAngularSpeedRadians * 4;
                public static final double BumperWidthMeters = Units.inchesToMeters(2.5);
                public static final double ApproachDistance = (WheelBaseMeters / 2) + BumperWidthMeters - 0.1;
        }

        public class Control {
                public static final double DriveDeadband = 0.15;
                public static final double DeadbandCurveWeight = 0.5;
        }

        // PID Constants
        public static final ExtendedPIDConstants DrivePID = new ExtendedPIDConstants(0.5, 0, 0, 0, 0.71, 0, 0.25);
        public static final ExtendedPIDConstants SteeringPID = new ExtendedPIDConstants(17, 0, 0);
        public static final ExtendedPIDConstants AutoAlignPID = new ExtendedPIDConstants(4, 0, 0.08);
        // Choreo trajectory following (feedback on top of the path's planned velocities)
        public static final ExtendedPIDConstants TrajectoryTranslationPID = new ExtendedPIDConstants(10, 0, 0);
        public static final ExtendedPIDConstants TrajectoryRotationPID = new ExtendedPIDConstants(7.5, 0, 0);

        // Uniform Drive Constants
        public static final double DriveGearRatio = 5.9;
        public static final double DriveWheelDiameterMeters = Units.inchesToMeters(3.875);
        public static final double DriveWheelCircumferenceMeters = Math.PI * DriveWheelDiameterMeters;
        public static final double SteeringGearRatio = 18.75;

        // Feature Flags
        // Default for the per-module "Swerve/Modules/<Module>/Optimize" tunable
        public static final boolean OptimizeModulesByDefault = false;
        public static final boolean USE_LIMELIGHT_POSE_ESTIMATION = false;

        // Drive motor open-loop voltage ramp (seconds from 0 to 12V)
        public static final double DriveMotorRampRate = 0.75;

        public static final int PigeonId = 13;

        // Modules, in the order used everywhere in the subsystem: FL, FR, RL, RR
        public static final ModuleConfig FrontLeftModule = new ModuleConfig("FrontLeft",
                        3, 4, 10, 0.064697, true, false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));
        public static final ModuleConfig FrontRightModule = new ModuleConfig("FrontRight",
                        6, 5, 11, 0.039307, true, false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, -Chassis.WheelBaseMeters / 2));
        public static final ModuleConfig RearLeftModule = new ModuleConfig("RearLeft",
                        7, 8, 12, 0.925781, false, false,
                        new Translation2d(-Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));
        public static final ModuleConfig RearRightModule = new ModuleConfig("RearRight",
                        2, 1, 9, 0.340332, false, false,
                        new Translation2d(-Chassis.TrackWidthMeters / 2, -Chassis.WheelBaseMeters / 2));

        public static final ModuleConfig[] Modules = {
                        FrontLeftModule, FrontRightModule, RearLeftModule, RearRightModule };
}
