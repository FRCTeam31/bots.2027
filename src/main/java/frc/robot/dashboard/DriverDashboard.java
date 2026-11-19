package frc.robot.dashboard;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.event.BooleanEvent;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.Tunables;

import frc.robot.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class DriverDashboard {
        private final String _fieldName = "Field";
        private final Field2d _fieldWidget;

        // Widgets
        private final HashMap<String, BooleanEvent> _booleanEvents = new HashMap<>();

        public DriverDashboard() {
                // Add complex data like the field view
                _fieldWidget = new Field2d();
                Tunables.publish(_fieldName, _fieldWidget);
        }

        public void setFieldRobotPose(Pose2d pose) {
                _fieldWidget.setRobotPose(pose);
        }

        public FieldObject2d getFieldTargetPose() {
                return _fieldWidget.getObject("target pose");
        }

        public void setFieldTargetPose(Pose2d pose) {
                getFieldTargetPose().setPose(pose);
        }

        public FieldObject2d getFieldVisionEstimationPose() {
                return _fieldWidget.getObject("VisionEstimation");
        }

        public void setFieldVisionEstimationPose(Pose2d pose) {
                getFieldVisionEstimationPose().setPose(pose);
        }

        public FieldObject2d getFieldPath() {
                return _fieldWidget.getObject("path");
        }

        public void setFieldPath(List<Pose2d> poses) {
                getFieldPath().setPoses(poses);
        }

        public void clearFieldPath() {
                getFieldPath().setPoses(new ArrayList<>());
        }

        /**
         * Gets a debounced event backed by a dashboard-editable boolean switch.
         * Calling this again with the same name returns the same event.
         */
        public BooleanEvent getSwitchBooleanEvent(String name, boolean defaultValue) {
                return _booleanEvents.computeIfAbsent(name, key -> {
                        TunableBoolean tunable = Tunables.addBoolean(key, defaultValue);
                        return new BooleanEvent(Robot.EventLoop, tunable).debounce(0.2);
                });
        }
}
