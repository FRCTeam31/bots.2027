package frc.robot.subsystems.swerve;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ImpactRumbleHelper;
import org.prime.control.SwerveControlSuppliers;
import org.prime.control.TunablePIDConstants;
import org.prime.subsystems.LoggedSubsystem;
import org.prime.sysid.SysIdRoutineHelper;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.CANBus;

import choreo.trajectory.SwerveSample;
import frc.robot.Container;
import frc.robot.Robot;

/**
 * Swerve drivetrain.
 *
 * <p>
 * Each loop:
 * <ol>
 * <li>{@link #periodic()} reads the gyro and the four {@link Module}s, then
 * updates
 * the pose estimate (odometry + vision).</li>
 * <li>A command (teleop driving, a trajectory, SysId) asks for chassis
 * velocities,
 * which {@link #driveRobotRelative} turns into one setpoint per module.</li>
 * </ol>
 *
 * <p>
 * Hardware is behind {@link GyroIO} and {@link ModuleIO}, so the same code runs
 * on the robot, in simulation, and in AdvantageKit log replay.
 */
public class Swerve extends LoggedSubsystem implements AutoCloseable {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  // Hardware
  private final GyroIO _gyroIO;
  private final GyroIOInputsAutoLogged _gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] _modules; // FL, FR, RL, RR (same order as SwerveMap.Modules)

  // Kinematics and pose estimation
  private final SwerveDriveKinematics _kinematics;
  private final SwerveDrivePoseEstimator _poseEstimator;
  private final VisionFusion _visionFusion = new VisionFusion();
  private ChassisVelocities _measuredVelocities = new ChassisVelocities();

  // AutoAlign: turns the robot to face a field-relative heading. The profile
  // limits
  // turn speed/acceleration so large turns are smooth instead of a full-power
  // snap.
  private final ProfiledPIDController _headingController = new ProfiledPIDController(
      SwerveMap.AutoAlignPID.kP,
      SwerveMap.AutoAlignPID.kI,
      SwerveMap.AutoAlignPID.kD,
      new TrapezoidProfile.Constraints(
          SwerveMap.Chassis.MaxAngularSpeedRadians,
          SwerveMap.Chassis.MaxAngularAccelerationRadians));
  private boolean _autoAlignEnabled = false;
  private double _autoAlignCorrection = 0;

  // Choreo trajectory following: feedback on top of the path's planned velocities
  private final PIDController _trajectoryXController = SwerveMap.TrajectoryTranslationPID
      .createPIDController(LOOP_PERIOD_SECONDS);
  private final PIDController _trajectoryYController = SwerveMap.TrajectoryTranslationPID
      .createPIDController(LOOP_PERIOD_SECONDS);
  private final PIDController _trajectoryHeadingController = SwerveMap.TrajectoryRotationPID
      .createPIDController(LOOP_PERIOD_SECONDS);

  // Dashboard-tunable gains
  private final TunablePIDConstants _driveGainsTuner = new TunablePIDConstants(
      "Swerve/DriveGains",
      SwerveMap.DrivePID);
  private final TunablePIDConstants _steerGainsTuner = new TunablePIDConstants(
      "Swerve/SteerGains",
      SwerveMap.SteeringPID);

  private final SysIdRoutineHelper _driveSysId;
  private final ImpactRumbleHelper _rumbleHelper = new ImpactRumbleHelper();

  /**
   * Creates the drivetrain with real, simulated, or replay hardware depending on
   * the robot mode.
   */
  public Swerve() {
    this(createGyroIO(), createModuleIOs());
  }

  /**
   * Creates the drivetrain with specific hardware. Used directly by tests.
   *
   * @param gyroIO    the gyro, or {@code null} to simulate one from module motion
   * @param moduleIOs four modules in FL, FR, RL, RR order
   */
  public Swerve(GyroIO gyroIO, ModuleIO... moduleIOs) {
    setName("Swerve");

    var configs = SwerveMap.Modules;
    _modules = new Module[configs.length];
    for (int i = 0; i < configs.length; i++) {
      _modules[i] = new Module(configs[i].name(), moduleIOs[i]);
    }

    _gyroIO = gyroIO != null
        ? gyroIO
        : new GyroIOSim(() -> _measuredVelocities.omega);

    _kinematics = new SwerveDriveKinematics(
        Arrays.stream(configs).map(ModuleConfig::location).toArray(Translation2d[]::new));

    // Read sensors once so the estimator starts from the real module positions
    updateSensors();
    _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, _gyroInputs.Yaw, getModulePositions(), Pose2d.ZERO);

    // 2027: Rotation2d angles are always wrapped to [-pi, pi], so heading
    // controllers
    // must treat the input as continuous to avoid turning the long way around
    _headingController.enableContinuousInput(-Math.PI, Math.PI);
    _headingController.setTolerance(Math.toRadians(1));
    _trajectoryHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // SysId drive characterization: all modules locked straight ahead, driven
    // together
    _driveSysId = new SysIdRoutineHelper(this, "SwerveDrive",
        voltage -> {
          for (var module : _modules) {
            module.runCharacterization(voltage.in(Volts), Rotation2d.ZERO);
          }
        },
        log -> {
          for (var module : _modules) {
            log.motor("drive-" + module.getName())
                .voltage(Volts.of(module.getDriveVoltage()))
                .linearPosition(Meters.of(module.getPosition().distance))
                .linearVelocity(MetersPerSecond.of(module.getVelocity().velocity));
          }
        });
  }

  private static GyroIO createGyroIO() {
    return switch (Robot.getMode()) {
      case REAL -> new GyroIOPigeon2(SwerveMap.PigeonId, new CANBus(CANPort.CAN_S1));
      case SIM -> null; // Simulated from module motion, see constructor
      case REPLAY -> new GyroIO() {
      };
    };
  }

  private static ModuleIO[] createModuleIOs() {
    var bus = new CANBus(CANPort.CAN_S1); // All drive hardware is on Systemcore CAN port S1
    return Arrays.stream(SwerveMap.Modules)
        .map(config -> switch (Robot.getMode()) {
          case REAL -> (ModuleIO) new ModuleIOTalonFX(config, bus);
          case SIM -> new ModuleIOSim();
          case REPLAY -> new ModuleIO() {
          };
        })
        .toArray(ModuleIO[]::new);
  }

  // #region Periodic

  @Override
  public void periodic() {
    updateSensors();
    _poseEstimator.update(_gyroInputs.Yaw, getModulePositions());
    _visionFusion.update(_measuredVelocities, _poseEstimator::addVisionMeasurement);

    applyTunedGains();
    _autoAlignCorrection = _autoAlignEnabled ? calculateHeadingCorrection() : 0;

    // Telemetry
    recordOutput("Pose", getPose());
    recordOutput("MeasuredModuleVelocities", getModuleVelocities());
    recordOutput("MeasuredChassisVelocities", _measuredVelocities);
    recordOutput("AutoAlign/Enabled", _autoAlignEnabled);
    recordOutput("AutoAlign/SetpointDegrees", Math.toDegrees(_headingController.getGoal().position));
    recordOutput("AutoAlign/AtSetpoint", _headingController.atGoal());
    recordOutput("AutoAlign/Correction", _autoAlignCorrection);
    if (Container.Dashboard != null) {
      Container.Dashboard.setFieldRobotPose(getPose());
    }

    updateDriverRumble();
  }

  /** Reads the gyro and modules, and computes the robot's measured velocity. */
  private void updateSensors() {
    _gyroIO.updateInputs(_gyroInputs);
    Logger.processInputs("Swerve/Gyro", _gyroInputs);

    for (var module : _modules) {
      module.periodic();
    }

    _measuredVelocities = _kinematics.toChassisVelocities(getModuleVelocities());
  }

  /** Pushes any gains edited on the dashboard to the modules. */
  private void applyTunedGains() {
    if (_driveGainsTuner.update()) {
      for (var module : _modules) {
        module.setDriveGains(SwerveMap.DrivePID);
      }
    }
    if (_steerGainsTuner.update()) {
      for (var module : _modules) {
        module.setSteerGains(SwerveMap.SteeringPID);
      }
    }
  }

  /** Rumbles the driver controller on hard impacts. */
  private void updateDriverRumble() {
    _rumbleHelper.addSample(_gyroInputs.AccelX, _gyroInputs.AccelY, _gyroInputs.AccelZ,
        _measuredVelocities.vx, SwerveMap.Chassis.MaxSpeedMetersPerSecond);

    var oi = Container.OperatorInterface;
    if (oi != null) {
      oi.setControllerRumbleIntensity(oi.DriverController, _rumbleHelper.getRumbleIntensity());
    }
  }

  // #endregion

  // #region Driving

  /**
   * Drives the robot. All other drive methods end up here.
   *
   * @param velocities robot-relative velocities (m/s, m/s, rad/s CCW+)
   */
  public void driveRobotRelative(ChassisVelocities velocities) {
    // AutoAlign takes over rotation while it's enabled
    var omega = _autoAlignEnabled ? _autoAlignCorrection : velocities.omega;

    // "Discretizing" compensates for the robot moving in straight segments each
    // loop, which otherwise causes drift while translating and rotating together
    var discretized = new ChassisVelocities(velocities.vx, velocities.vy, omega).discretize(LOOP_PERIOD_SECONDS);
    var setpoints = _kinematics.toSwerveModuleVelocities(discretized);

    recordOutput("DesiredChassisVelocities", discretized);
    recordOutput("DesiredModuleVelocities", setpoints);
    for (int i = 0; i < _modules.length; i++) {
      _modules[i].runSetpoint(setpoints[i]);
    }
  }

  /** Follows one sample of a Choreo trajectory. */
  public void followTrajectory(SwerveSample sample) {
    var pose = getPose();
    var fieldRelative = new ChassisVelocities(
        sample.vx + _trajectoryXController.calculate(pose.getX(), sample.x),
        sample.vy + _trajectoryYController.calculate(pose.getY(), sample.y),
        sample.omega + _trajectoryHeadingController.calculate(pose.getRotation().getRadians(), sample.heading));

    driveRobotRelative(fieldRelative.toRobotRelative(pose.getRotation()));
  }

  public void stop() {
    for (var module : _modules) {
      module.stop();
    }
  }

  // #endregion

  // #region Pose and heading

  /** The best estimate of where the robot is on the field (odometry + vision). */
  public Pose2d getPose() {
    return _poseEstimator.getEstimatedPosition();
  }

  /** Moves the pose estimate to {@code pose}, e.g. at the start of an auto. */
  public void setPose(Pose2d pose) {
    _poseEstimator.resetPosition(_gyroInputs.Yaw, getModulePositions(), pose);
  }

  /** The gyro heading used for field-relative driving. */
  public Rotation2d getGyroHeading() {
    return _gyroInputs.Yaw;
  }

  /** Measured robot-relative velocity. */
  public ChassisVelocities getMeasuredVelocities() {
    return _measuredVelocities;
  }

  /**
   * Re-zeroes the gyro so "forward" points away from the driver station
   * (180 degrees on blue). The field pose estimate is kept.
   */
  public void resetGyro() {
    var newYaw = Rotation2d.fromDegrees(Robot.onBlueAlliance() ? 180 : 0);
    _gyroIO.setYaw(newYaw);
    _gyroInputs.Yaw = newYaw;
    _poseEstimator.resetPosition(newYaw, getModulePositions(), getPose());
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(_modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  private SwerveModuleVelocity[] getModuleVelocities() {
    return Arrays.stream(_modules).map(Module::getVelocity).toArray(SwerveModuleVelocity[]::new);
  }

  // #endregion

  // #region AutoAlign

  /**
   * Turns on AutoAlign toward a field-relative heading (degrees; flipped for blue
   * alliance).
   */
  private void enableAutoAlign(double angleDegrees) {
    var setpoint = Robot.onBlueAlliance() ? angleDegrees + 180 : angleDegrees;

    // Start the motion profile from the robot's current heading and turn rate
    _headingController.reset(getGyroHeading().getRadians(), _measuredVelocities.omega);
    _headingController.setGoal(Rotation2d.fromDegrees(setpoint).getRadians());
    _autoAlignEnabled = true;
  }

  private void disableAutoAlign() {
    _autoAlignEnabled = false;
  }

  /**
   * The turn rate (rad/s) that follows the heading profile toward the AutoAlign
   * setpoint.
   */
  private double calculateHeadingCorrection() {
    var feedback = _headingController.calculate(getGyroHeading().getRadians());
    // The profile's planned velocity is a feedforward; the PID only corrects error
    var correction = feedback + _headingController.getSetpoint().velocity;
    return Math.clamp(correction, -SwerveMap.Chassis.MaxAngularSpeedRadians,
        SwerveMap.Chassis.MaxAngularSpeedRadians);
  }

  public boolean isAutoAlignEnabled() {
    return _autoAlignEnabled;
  }

  public boolean isAutoAlignAtSetpoint() {
    return _headingController.atGoal();
  }

  // #endregion

  // #region Commands

  /**
   * Drives field-relative from controller input. Turning the rotation stick
   * cancels AutoAlign.
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controls) {
    return this.run(() -> driveRobotRelative(
        controls.getChassisVelocities(false, getGyroHeading(), this::disableAutoAlign)))
        .handleInterrupt(() -> DriverStationErrors.reportWarning("[DRIVE] Default command interrupted", false));
  }

  public Command stopAllMotorsCommand() {
    return this.runOnce(this::stop);
  }

  public Command resetGyroCommand() {
    return Commands.runOnce(this::resetGyro);
  }

  /** Enables AutoAlign toward a heading (degrees). */
  public Command setAutoAlignSetpointCommand(double angleDegrees) {
    return Commands.runOnce(() -> enableAutoAlign(angleDegrees));
  }

  public Command disableAutoAlignCommand() {
    return Commands.runOnce(this::disableAutoAlign).ignoringDisable(true).withName("DisableAutoAlign");
  }

  /**
   * SysId characterization of the drive motors (modules locked straight ahead).
   */
  public Command sysIdDriveCommand(SysIdRoutineHelper.TestType testType,
      SysIdRoutineHelper.TestDirection direction) {
    return _driveSysId.getCommand(testType, direction);
  }

  // #endregion

  /**
   * Releases resources with unique IDs (2027 Alerts) so another instance can be
   * created, e.g. in tests.
   */
  @Override
  public void close() {
    _visionFusion.close();
  }
}
