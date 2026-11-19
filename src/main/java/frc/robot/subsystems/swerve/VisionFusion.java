package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import frc.robot.Container;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightCameraInputs;
import frc.robot.subsystems.vision.photon.PhotonCameraInputs;

/**
 * Decides which camera pose estimates are trustworthy enough to correct the
 * drivetrain's odometry.
 */
public class VisionFusion implements AutoCloseable {

  /** Receives an accepted vision measurement (e.g. the pose estimator). */
  @FunctionalInterface
  public interface MeasurementConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  // Vision is ignored while moving faster than this, because motion blur and
  // latency make estimates unreliable
  private static final double MAX_LINEAR_SPEED_MPS = 2.0;
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 1.0; // ~60 deg/s

  private final Alert _photonTooFarAlert = new Alert("photonEstimationSkipped", "Photon estimation skipped", Level.LOW);

  /** Checks every camera and passes trustworthy estimates to {@code consumer}. */
  public void update(ChassisVelocities robotVelocities, MeasurementConsumer consumer) {
    var slowEnough = Math.abs(robotVelocities.vx) < MAX_LINEAR_SPEED_MPS
        && Math.abs(robotVelocities.vy) < MAX_LINEAR_SPEED_MPS
        && Math.abs(robotVelocities.omega) < MAX_ANGULAR_SPEED_RAD_PER_SEC;

    Logger.recordOutput("Swerve/Vision/WithinPoseEstimationVelocity", slowEnough);
    if (!slowEnough) {
      return;
    }

    var limelight = Container.LimelightVision;
    if (SwerveMap.USE_LIMELIGHT_POSE_ESTIMATION) {
      addLimelightEstimate(limelight.State.get(VisionMap.LimelightName), consumer);
    }

    var photon = Container.PhotonVision;
    if (photon != null) {
      for (var camera : new String[] { VisionMap.PhotonCam1Name, VisionMap.PhotonCam2Name }) {
        if (photon.State.containsKey(camera)) {
          addPhotonEstimate(photon.State.get(camera), consumer);
        }
      }
    }
  }

  private void addLimelightEstimate(LimelightCameraInputs inputs, MeasurementConsumer consumer) {
    var estimate = inputs.BotPoseEstimate;
    if (estimate == null || estimate.tagCount == 0) {
      return;
    }

    // A single tag is only trusted if it is a real field tag, unambiguous, and
    // close
    if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
      var tag = estimate.rawFiducials[0];
      boolean isValidTag = tag.id >= 1 && tag.id <= 22;
      boolean tooAmbiguous = tag.ambiguity > 0.7;
      boolean tooFar = tag.distToCamera > 3;
      if (!isValidTag || tooAmbiguous || tooFar) {
        return;
      }
    }

    // Trust X/Y but not the heading (the gyro is much better at that)
    consumer.accept(estimate.pose, estimate.timestampSeconds, VecBuilder.fill(0.5, 0.5, 9999999));
  }

  private void addPhotonEstimate(PhotonCameraInputs inputs, MeasurementConsumer consumer) {
    if (inputs.BotPoseEstimate == null || inputs.TargetCount == 0) {
      return;
    }

    var tooFar = inputs.AverageTagDistance > VisionMap.PHOTON_MAX_AVG_TAG_DISTANCE_METERS;
    _photonTooFarAlert.set(tooFar);
    if (tooFar) {
      Logger.recordOutput("Swerve/Vision/RejectedPhotonTagDistance", inputs.AverageTagDistance);
      return;
    }

    consumer.accept(inputs.BotPoseEstimate, inputs.TimestampSeconds,
        VecBuilder.fill(inputs.CurrentStdDevs[0], inputs.CurrentStdDevs[1], inputs.CurrentStdDevs[2]));
  }

  @Override
  public void close() {
    _photonTooFarAlert.close();
  }
}
