package org.prime.control;

import org.prime.control.CommandXboxController;
import org.wpilib.driverstation.XboxController.Axis;

import java.util.function.DoubleSupplier;

public class SupplierXboxController extends CommandXboxController {

  public SupplierXboxController(int port) {
    super(port);
  }

  /**
   * Returns a cubic-scaled supplier for the left stick's Y axis with a deadband
   * and curve weight
   * 
   * @param deadband    (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getLeftStickYSupplier(double deadband, double curveWeight) {
    return () -> -Controls.cubicScaledDeadband(Controller.getAxis(Axis.LEFT_Y), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the left stick's Y axis with a deadband
   * 
   * @param deadband (0-1)
   */
  public DoubleSupplier getLeftStickYSupplier(double deadband) {
    return () -> -Controls.linearScaledDeadband(Controller.getAxis(Axis.LEFT_Y), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the left stick's X axis with a deadband
   * and curve weight
   * 
   * @param deadband    (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getLeftStickXSupplier(double deadband, double curveWeight) {
    return () -> Controls.cubicScaledDeadband(Controller.getAxis(Axis.LEFT_X), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the left stick's X axis with a deadband
   * 
   * @param deadband (0-1)
   */
  public DoubleSupplier getLeftStickXSupplier(double deadband) {
    return () -> Controls.linearScaledDeadband(Controller.getAxis(Axis.LEFT_X), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the right stick's Y axis with a deadband
   * and curve weight
   * 
   * @param deadband    (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getRightStickYSupplier(double deadband, double curveWeight) {
    return () -> -Controls.cubicScaledDeadband(Controller.getAxis(Axis.RIGHT_Y), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the right stick's Y axis with a deadband
   * 
   * @param deadband (0-1)
   */
  public DoubleSupplier getRightStickYSupplier(double deadband) {
    return () -> -Controls.linearScaledDeadband(Controller.getAxis(Axis.RIGHT_Y), deadband);
  }

  /**
   * Returns a cubic-scaled supplier for the right stick's X axis with a deadband
   * and curve weight
   * 
   * @param deadband    (0-1)
   * @param curveWeight (0-1)
   */
  public DoubleSupplier getRightStickXSupplier(double deadband, double curveWeight) {
    return () -> Controls.cubicScaledDeadband(Controller.getAxis(Axis.RIGHT_X), deadband, curveWeight);
  }

  /**
   * Returns a linear-scaled supplier for the right stick's X axis with a deadband
   * 
   * @param deadband (0-1)
   */
  public DoubleSupplier getRightStickXSupplier(double deadband) {
    return () -> Controls.linearScaledDeadband(Controller.getAxis(Axis.RIGHT_X), deadband);
  }

  /**
   * Returns a supplier for the trigger axis with the left trigger subtracted from
   * the right trigger
   */
  public DoubleSupplier getTriggerSupplier(double deadband, double curveWeight) {
    return () -> Controls.cubicScaledDeadband(Controller.getAxis(Axis.RIGHT_TRIGGER), deadband, curveWeight) -
        Controls.cubicScaledDeadband(Controller.getAxis(Axis.LEFT_TRIGGER), deadband, curveWeight);
  }

  /**
   * Returns a supplier for the trigger axis with the right trigger subtracted
   * from the left trigger
   */
  public DoubleSupplier getInvertedTriggerSupplier() {
    return () -> Controller.getAxis(Axis.LEFT_TRIGGER) - Controller.getAxis(Axis.RIGHT_TRIGGER);
  }

  /**
   * Returns cubic-scaled suppliers for the SwerveDrive control inputs
   * 
   * @param style
   * @param driveDeadband
   * @param curveWeight
   * @return
   */
  public SwerveControlSuppliers getSwerveControlProfile(
      HolonomicControlStyle style,
      double driveDeadband,
      double curveWeight) {
    switch (style) {
      default:
      case Standard:
        return new SwerveControlSuppliers(
            getLeftStickXSupplier(driveDeadband, curveWeight),
            getLeftStickYSupplier(driveDeadband, curveWeight),
            getTriggerSupplier(driveDeadband, curveWeight));
      case Drone:
        return new SwerveControlSuppliers(
            getRightStickXSupplier(driveDeadband, curveWeight),
            getRightStickYSupplier(driveDeadband, curveWeight),
            getLeftStickXSupplier(driveDeadband, curveWeight));
      case DroneReversed:
        return new SwerveControlSuppliers(
            getLeftStickXSupplier(driveDeadband, curveWeight),
            getLeftStickYSupplier(driveDeadband, curveWeight),
            getRightStickXSupplier(driveDeadband, curveWeight));
    }
  }
}
