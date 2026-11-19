package org.prime.control;

import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

/**
 * Publishes an {@link ExtendedPIDConstants} to the dashboard as tunables
 * ({@code <path>/kP}, {@code <path>/kI}, ...) so gains can be edited live.
 *
 * <pre>
 * var tuner = new TunablePIDConstants("Swerve/DriveGains", SwerveMap.DrivePID);
 * // in periodic():
 * if (tuner.update()) {
 *   motor.applyGains(SwerveMap.DrivePID);
 * }
 * </pre>
 */
public class TunablePIDConstants {
  private final ExtendedPIDConstants _constants;
  private final TunableDouble _kP, _kI, _kD, _kS, _kV, _kA;

  public TunablePIDConstants(String path, ExtendedPIDConstants constants) {
    _constants = constants;
    _kP = Tunables.addDouble(path + "/kP", constants.kP);
    _kI = Tunables.addDouble(path + "/kI", constants.kI);
    _kD = Tunables.addDouble(path + "/kD", constants.kD);
    _kS = Tunables.addDouble(path + "/kS", constants.kS);
    _kV = Tunables.addDouble(path + "/kV", constants.kV);
    _kA = Tunables.addDouble(path + "/kA", constants.kA);
  }

  /**
   * Copies any dashboard edits into the constants object.
   *
   * @return true if any gain changed since the last call
   */
  public boolean update() {
    var changed = _kP.get() != _constants.kP || _kI.get() != _constants.kI || _kD.get() != _constants.kD
        || _kS.get() != _constants.kS || _kV.get() != _constants.kV || _kA.get() != _constants.kA;

    if (changed) {
      _constants.kP = _kP.get();
      _constants.kI = _kI.get();
      _constants.kD = _kD.get();
      _constants.kS = _kS.get();
      _constants.kV = _kV.get();
      _constants.kA = _kA.get();
    }
    return changed;
  }
}
