package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

import frc.robot.subsystems.vision.photon.PhotonVision;

import org.wpilib.command2.Command;

public interface IVisionSubsystem<Type> {
    Type addCamera(String name, Transform3d robotCameraTransform);

    boolean removeCamera(String name);

    java.util.Set<String> getCameraNames();

    boolean hasCamera(String name);

    void setPipeline(String name, int pipeline);

    void setCameraPose(String name, Pose3d pose);

    void periodic();

    Command setProcessingPipeline(String name, int pipeline);
}
