package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.Container;

public class BotControls {
        public static class OIMap {
                // public static final HolonomicControlStyle DefaultDriveControlStyle =
                // HolonomicControlStyle.Drone;
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.DroneReversed;
        }

        public SupplierXboxController DriverController;
        public SupplierXboxController OperatorController;

        public BotControls() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        public void bindDriverControls() {
                // Driver Controls:
                // Swerve controls with sticks
                // A - Reset gyro

                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                // Serve controls
                Container.Swerve.setDefaultCommand(Container.Swerve.driveFieldRelativeCommand(controlProfile));

                // Reset gyro
                DriverController.a()
                                .onTrue(Container.Swerve.resetGyroCommand());
        }

        public void bindOperatorControls() {
                // Operator Controls:
                // RT - Shoot (uptake + firing + transfer)
                // RB - Transfer feed in
                // LB - Set turret auto
                // Y - Intake out position + auto feed
                // B - Intake in position + stop feed after 1.5s
                // X - E-Stop intake rollers
                // Up d-pad - Flywheel speed up / manual mode
                // Down d-pad - Flywheel speed down / manual mode
                // Left d-pad - Turret yaw CCW / manual mode
                // Right d-pad - Turret yaw CW / manual mode

                // Container.Turret.setDefaultCommand(Container.Turret
                // .controlTurretYawWithStick(OperatorController.getLeftStickXSupplier(0)));

                // OperatorController.rightTrigger()
                // .onTrue(Container.Turret.setFeed(UptakeState.FORWARDS)
                // .andThen(Container.Turret.setFiring(FiringState.FIRING))
                // .andThen(Container.Hopper.setFeed(TransferFeedState.INWARDS)))
                // //
                // .andThen(Container.Hopper.oscillateIntake(HopperMap.INTAKE_OSCILLATION_CYCLE_SECONDS)))
                // .onFalse(Container.Turret.setFeed(UptakeState.STOPPED)
                // .andThen(Container.Turret.setFiring(FiringState.FIRING))
                // .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                // OperatorController.rightBumper().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS))
                // .onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                // OperatorController.y().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT)
                // .andThen(Container.Hopper.setIntakeFeedSupplier(() -> _automaticFeedState)));
                // OperatorController.b().onTrue(
                // Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN)
                // .andThen(Commands.waitSeconds(1.5))
                // .andThen(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED))
                // .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                // OperatorController.x().onTrue(Commands
                // .runOnce(() -> _automaticFeedState = _automaticFeedState !=
                // IntakeFeedState.STOPPED
                // ? IntakeFeedState.STOPPED
                // : IntakeFeedState.INWARDS));

                // // Controls to toggle Turret auto and manual
                // OperatorController.leftBumper()
                // .onTrue(Container.Turret.setOperatingMode(OperatingMode.AUTO));

                // // Manual turret controls
                // OperatorController.dpadRight()
                // .onTrue(Container.Turret.adjustManualYaw(-TurretMap.MANUAL_YAW_STEP_DEGREES));
                // OperatorController.dpadLeft()
                // .onTrue(Container.Turret.adjustManualYaw(TurretMap.MANUAL_YAW_STEP_DEGREES));

                // OperatorController.dpadUp()
                // .onTrue(Container.Turret
                // .adjustManualFlywheelSpeed(TurretMap.MANUAL_FLYWHEEL_STEP_RPS));
                // OperatorController.dpadDown()
                // .onTrue(Container.Turret
                // .adjustManualFlywheelSpeed(-TurretMap.MANUAL_FLYWHEEL_STEP_RPS));

                // OperatorController.leftStick().and(OperatorController.rightStick())
                // .onTrue(Container.Turret.setOperatingMode(OperatingMode.STOPPED));

                // OperatorController.a().onTrue(Container.Turret.adjustManualHoodAngle(-0.1));
                // OperatorController.x().onTrue(Container.Turret.adjustManualHoodAngle(0.1));
        }

        public void setControllerRumbleIntensity(SupplierXboxController controller, double intensity) {
                controller.setRumble(intensity);
        }

        public Command rumbleControllerShort(SupplierXboxController controller) {
                return Commands.runOnce(() -> controller.setRumble(1))
                                .andThen(Commands.waitSeconds(0.2))
                                .andThen(Commands.runOnce(() -> controller.setRumble(0)));
        }

        /**
         * Contains custom commands used to control multiple subsystems.
         */
        public class MultiSysCommands {
                // /**
                // * Command to set the hopper and intake into the correct positions
                // * when starting a match
                // *
                // * @return Command
                // */
                // public static Command homeRobotCommand() {
                // return Container.Hopper.setFeed(TransferFeedState.INWARDS)
                // .andThen(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT))
                // // Time for intake to fully extend
                // .andThen(Commands.waitSeconds(1))
                // .andThen(Container.Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
                // }

                // public static Command setIntakeStates(IntakeCombinedState state) {
                // if (state == IntakeCombinedState.INWARDS) {
                // return Container.Hopper.setIntakeFeed(IntakeFeedState.INWARDS)
                // .andThen(Container.Hopper.setFeed(TransferFeedState.INWARDS))
                // // .andThen(Container.Turret.setFeed(UptakeState.FORWARDS))
                // .withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING);
                // } else {
                // return Container.Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS)
                // .andThen(Container.Hopper.setFeed(TransferFeedState.OUTWARDS))
                // // .andThen(Container.Turret.setFeed(UptakeState.REVERSED))
                // .withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING);
                // }
                // }

                // public static Command toggleShooterOn() {
                // return Turret.setFiring(FiringState.FIRING)
                // .andThen(Commands.waitUntil(() -> SuperStructure.Turret.FlywheelAtTargetSpeed
                // &&
                // SuperStructure.Turret.YawOnTarget &&
                // SuperStructure.Turret.HoodOnTarget))
                // .andThen(Turret.setFeed(UptakeState.FORWARDS))
                // .andThen(() -> SuperStructure.Hopper.TransferFeedState =
                // TransferFeedState.INWARDS);
                // }

                // public static Command toggleShooterOff() {
                // return Turret.setFiring(FiringState.IDLE)
                // .andThen(Turret.setFeed(UptakeState.STOPPED))
                // .andThen(() -> SuperStructure.Hopper.TransferFeedState =
                // TransferFeedState.STOPPED);
                // }

                // public static Command startAuto() {
                // return Turret.setOperatingMode(OperatingMode.AUTO)
                // .andThen(Turret.setFiring(FiringState.IDLE));
                // }

                // public static Command toggleIntakeOn() {
                // return Commands.runOnce(() -> Container.Hopper.State.IntakeControlState =
                // HopperIntakeState.OUT)
                // .andThen(() -> Container.Hopper.State.IntakeFeedState =
                // IntakeFeedState.INWARDS)
                // .andThen(() -> Container.Hopper.State.TransferFeedState =
                // TransferFeedState.STOPPED); // These
                // // have
                // // been
                // // changed
                // // for
                // // our
                // // current
                // // strategy
                // // for
                // // defense
                // }

                // public static Command toggleIntakeOff() {
                // return Commands.runOnce(() -> Container.Hopper.State.IntakeControlState =
                // HopperIntakeState.IN)
                // .andThen(() -> Container.Hopper.State.IntakeFeedState =
                // IntakeFeedState.STOPPED);
                // }

                // public static Command ejectBalls() {
                // return Commands.runOnce(() -> Container.Hopper.State.IntakeFeedState =
                // IntakeFeedState.OUTWARDS)
                // .andThen(() -> Container.Hopper.State.TransferFeedState =
                // TransferFeedState.OUTWARDS);
                // }

                // public static Command toggleIntakeOff() {
                // return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState =
                // HopperIntakeState.IN)
                // .andThen(() -> SuperStructure.Turret.FeedState = UptakeState.STOPPED);
                // }

                // public static List<Pair<String, Command>> getNamedCommandSuppliers() {
                // return List.of(
                // // Pair.of("Disable_Autonomous_Shooting", toggleShooterOff()),
                // // Pair.of("Start_Auto", startAuto()),
                // Pair.of("Take_Out_And_Enable_Intake", toggleIntakeOn()),
                // Pair.of("Put_In_And_Disable_Intake", toggleIntakeOff()),
                // Pair.of("Dump_Balls", ejectBalls()));
                // }
        }
}
