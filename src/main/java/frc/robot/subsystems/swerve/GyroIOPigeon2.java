package frc.robot.subsystems.swerve;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

/** Real gyro: CTRE Pigeon 2.0 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 _pigeon;
  private final StatusSignal<Angle> _yaw;
  private final StatusSignal<AngularVelocity> _yawVelocity;
  private final StatusSignal<LinearAcceleration> _accelX, _accelY, _accelZ;

  public GyroIOPigeon2(int canId, CANBus bus) {
    _pigeon = new Pigeon2(canId, bus);
    _pigeon.getConfigurator().apply(new Pigeon2Configuration());

    _yaw = _pigeon.getYaw();
    _yawVelocity = _pigeon.getAngularVelocityZWorld();
    _accelX = _pigeon.getAccelerationX();
    _accelY = _pigeon.getAccelerationY();
    _accelZ = _pigeon.getAccelerationZ();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(_yaw, _yawVelocity, _accelX, _accelY, _accelZ);
    inputs.Connected = status.isOK();
    inputs.Yaw = Rotation2d.fromDegrees(_yaw.getValue().in(Degrees));
    inputs.YawVelocityRadPerSec = _yawVelocity.getValue().in(RadiansPerSecond);
    inputs.AccelX = _accelX.getValueAsDouble();
    inputs.AccelY = _accelY.getValueAsDouble();
    inputs.AccelZ = _accelZ.getValueAsDouble();
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    var status = _pigeon.setYaw(yaw.getDegrees());
    DriverStationErrors.reportWarning("Reset gyro to " + yaw.getDegrees() + ": " + status, false);
  }
}
