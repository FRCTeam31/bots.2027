package frc.robot.subsystems.swerve;

import org.wpilib.math.geometry.Translation2d;

/**
 * Per-module hardware configuration.
 *
 * @param name                   Name used for logging and tunables (e.g. "FrontLeft")
 * @param driveMotorId           CAN ID of the drive TalonFX
 * @param steerMotorId           CAN ID of the steering TalonFX
 * @param encoderId              CAN ID of the CANcoder
 * @param encoderOffsetRotations CANcoder reading (rotations) when the wheel points forward
 * @param driveInverted          Whether the drive motor is inverted
 * @param steerInverted          Whether the steering motor is inverted
 * @param location               Module position relative to robot center (meters, +X forward, +Y left)
 */
public record ModuleConfig(
    String name,
    int driveMotorId,
    int steerMotorId,
    int encoderId,
    double encoderOffsetRotations,
    boolean driveInverted,
    boolean steerInverted,
    Translation2d location) {
}
