package org.prime.util;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.math.trajectory.Trajectory;

public class PoseUtil {
        public static HolonomicTrajectory generateStraightLineTrajectory(Pose2d startPose,
                        double distanceMeters, int durationSeconds) {
                // Extract heading from pose
                Rotation2d heading = startPose.getRotation();

                // Compute end position
                Translation2d endTranslation = new Translation2d(
                                startPose.getX() + distanceMeters * heading.getCos(),
                                startPose.getY() + distanceMeters * heading.getSin());

                Pose2d endPose = new Pose2d(endTranslation, heading);

                var startSample = new HolonomicSample(0, startPose, new ChassisVelocities(),
                                new ChassisAccelerations());
                var endSample = new HolonomicSample(durationSeconds, endPose, new ChassisVelocities(),
                                new ChassisAccelerations());

                // Generate trajectory
                return new HolonomicTrajectory(List.of(startSample, endSample));
        }

        /**
         * Gets the poses from a trajectory.
         * 
         * @param trajectory
         * @return
         */
        public static List<Pose2d> getTrajectoryPoses(Trajectory<HolonomicSample> trajectory) {
                var samples = trajectory.getSamples();
                var poses = new ArrayList<Pose2d>(samples.size());
                for (var sample : samples) {
                        poses.add(sample.pose);
                }

                return poses;
        }

        /**
         * Gets the distance between two poses.
         * 
         * @param pose1
         * @param pose2
         * @return
         */
        public static double getDistanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
                return pose1.getTranslation().getDistance(pose2.getTranslation());
        }

        /**
         * Returns the closest pose in the list to the current pose, or null if the list
         * is empty.
         * 
         * @param currentPose
         * @param poses
         * @return
         */
        public static Pose2d getClosestPoseInList(Pose2d currentPose, List<Pose2d> poses) {
                return poses.stream()
                                .min((pose1, pose2) -> Double.compare(
                                                getDistanceBetweenPoses(currentPose, pose1),
                                                getDistanceBetweenPoses(currentPose, pose2)))
                                .orElse(null);
        }

        /**
         * Converts a pose from robot space to field space.
         * 
         * @param robotPose_Field  The robot's pose in field space.
         * @param targetPose_Robot The target pose in robot space.
         * @return
         */
        public static Pose2d convertPoseFromRobotToFieldSpace(Pose2d robotPose_Field, Pose2d targetPose_Robot) {
                var transform = new Transform2d(targetPose_Robot.getTranslation(), targetPose_Robot.getRotation());

                return robotPose_Field.transformBy(transform);
        }
}
