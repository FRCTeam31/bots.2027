// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.prime.dashboard.Elastic;
import org.prime.util.BuildConstants;

import org.wpilib.net.WebServer;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.Units;
import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.event.BooleanEvent;
import org.wpilib.event.EventLoop;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.subsystems.leds.LEDPatterns;
import frc.robot.subsystems.leds.MatchLEDHelper;

public class Robot extends LoggedRobot {

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Mode to use when not on a real robot. Change to REPLAY to replay a log in AdvantageKit. */
  public static final Mode SIM_MODE = Mode.SIM;

  /** Whether hardware is real, simulated, or replayed from a log. */
  public static Mode getMode() {
    return isReal() ? Mode.REAL : SIM_MODE;
  }

  public static EventLoop EventLoop = new EventLoop();
  private Command _autonomousCommand;
  private boolean _hasEnteredTeleop = false;

  public Robot() {
    // Configure logging
    configureLogging();
    // DriverStation.silenceJoystickConnectionWarning(true);

    // Set up the dashboard
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath()); // Start the web server for downloading elastic
                                                                      // layout from robot
    Elastic.selectTab("Auto");
    Preferences.removeAll();

    // Set an LED pattern to display when the robot goes disabled after a match
    new BooleanEvent(EventLoop, () -> RobotState.isDisabled() &&
        _hasEnteredTeleop)
        .rising()
        .ifHigh(() -> CommandScheduler.getInstance().schedule(Commands.sequence(
            Container.LEDs.setAllSectionPatternsCommand(
                LEDPatterns.fastBlink(Color.GREEN, Units.Milliseconds.of(50))),
            Commands.waitSeconds(0.5),
            Container.LEDs
                .setAllSectionPatternsCommand(LEDPattern.solid(getAllianceColor()).breathe(Units.Seconds.of(3))))
            .ignoringDisable(true)));
  }

  /**
   * Logs program metadata, configures logging, and AdvantageKit data receivers,
   * and configures robot logging mode.
   */
  private void configureLogging() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Event", MatchState.getEventName());
    Logger.recordMetadata("Match Type", MatchState.getMatchType().toString());
    Logger.recordMetadata("Match Number", String.valueOf(MatchState.getMatchNumber()));

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("Robot disabled");
    CommandScheduler.getInstance().schedule(Container.Swerve.disableAutoAlignCommand());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    EventLoop.poll();

    // Update match state LED strip
    MatchLEDHelper.updateLEDs();
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    // Cancel any auto command that's still running
    if (_autonomousCommand != null)
      _autonomousCommand.cancel();

    // _autonomousCommand = Container.AutoChooser.getSelected();

    if (_autonomousCommand == null || _autonomousCommand == Commands.none()) {
      DriverStationErrors.reportError("[ERROR] >> No auto command selected", false);
    } else {
      // Reset the gyro immediately if we're on the red alliance to get the correct
      // angle (reversed on red alliance)
      if (onRedAlliance()) {
        Container.Swerve.resetGyro();
      }

      // Schedule the auto command
      CommandScheduler.getInstance().schedule(_autonomousCommand);
    }
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    DataLogManager.log("Teleop Enabled");
    _hasEnteredTeleop = true;

    if (_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      _autonomousCommand.cancel();
    }
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public static boolean onRedAlliance() {
    var alliance = MatchState.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.RED;
  }

  public static boolean onBlueAlliance() {
    var alliance = MatchState.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.BLUE;
  }

  public static Color getAllianceColor() {
    var alliance = MatchState.getAlliance();
    Color allianceColor = Color.GHOST_WHITE;
    if (alliance.isPresent())
      allianceColor = alliance.get() == Alliance.RED
          ? Color.RED
          : Color.BLUE;

    return allianceColor;
  }
}
