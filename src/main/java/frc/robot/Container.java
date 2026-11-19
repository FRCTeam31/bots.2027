// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoChooser;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.tunable.Tunables;
import frc.robot.dashboard.DriverDashboard;
import frc.robot.oi.BotControls;
import frc.robot.pneumatics.Pneumatics;
import frc.robot.subsystems.leds.ILEDs;
import frc.robot.subsystems.prism.IPrism;
import frc.robot.subsystems.prism.PrismMap;
import frc.robot.subsystems.prism.PrismReal;
import frc.robot.subsystems.prism.PrismSim;
import frc.robot.subsystems.prism.PrismSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightVision;
import frc.robot.subsystems.vision.photon.PhotonVision;

public class Container {
  public static final DriverDashboard Dashboard = new DriverDashboard();
  public static final BotControls OperatorInterface = new BotControls();

  public static ILEDs LEDs;
  public static Swerve Swerve;
  public static AutoFactory AutoFactory;
  public static AutoChooser AutoChooser;
  public static LimelightVision LimelightVision;
  public static PhotonVision PhotonVision;
  public static Pneumatics Pneumatics;
  public static PowerDistribution PD;

  static {
    try {
      // Create subsystems
      LEDs = new PrismSubsystem(
          switch (Robot.getMode()) {
            case REAL -> new PrismReal(PrismMap.REAL_SERIAL_PORT);
            case SIM -> new PrismSim(PrismMap.SIM_COM_PORT, PrismMap.SIM_BAUD_RATE);
            case REPLAY -> new IPrism() {
            };
          },
          PrismMap.STRIP_CONFIGS);

      LimelightVision = new LimelightVision()
          .addCamera(VisionMap.LimelightName, VisionMap.LimelightTransform);
      PhotonVision = new PhotonVision()
          .addCamera(VisionMap.PhotonCam1Name, VisionMap.PhotonCam1Transform)
          .addCamera(VisionMap.PhotonCam2Name, VisionMap.PhotonCam2Transform);

      Swerve = new Swerve();
      // AutoFactory = new AutoFactory(
      // () -> Swerve.State.EstimatedRobotPose, // A function that returns the current
      // robot pose
      // Swerve::resetOdometry, // A function that resets the current robot pose to
      // the provided Pose2d
      // Swerve::followTrajectory, // The drive subsystem trajectory follower
      // true, // If alliance flipping should be enabled
      // Swerve // The drive subsystem
      // );
      // Build an auto chooser. This will use Commands.none() as the default option.
      AutoChooser = new AutoChooser();
      Tunables.publish("Auto", AutoChooser);

      Pneumatics = new Pneumatics();
      PD = new PowerDistribution(CANPort.CAN_S0, 1, ModuleType.REV);

      // Create and bind the operator interface
      OperatorInterface.bindDriverControls();
      OperatorInterface.bindOperatorControls();

    } catch (Exception e) {
      DriverStationErrors.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(),
          e.getStackTrace());
    }
  }

  // #region Commands

}