package org.prime.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.util.WPISerializable;
import org.wpilib.util.struct.StructSerializable;
import org.wpilib.command2.SubsystemBase;

public class LoggedSubsystem extends SubsystemBase {

    protected void processInputs(LoggableInputs inputs) {
        Logger.processInputs(getName(), inputs);
    }

    protected void recordOutput(String keyName, byte value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected void recordOutput(String keyName, boolean value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected void recordOutput(String keyName, int value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected void recordOutput(String keyName, long value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected void recordOutput(String keyName, float value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected void recordOutput(String keyName, double value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected <E extends Enum<E>> void recordOutput(String key, E value) {
        Logger.recordOutput(getName() + "/" + key, value);
    }

    protected <T extends WPISerializable> void recordOutput(String keyName, T value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected <T extends StructSerializable> void recordOutput(String keyName, T value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    @SuppressWarnings("unchecked")
    protected <T extends StructSerializable> void recordOutput(String keyName, T... value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

    protected <T extends Record> void recordOutput(String keyName, T value) {
        Logger.recordOutput(getName() + "/" + keyName, value);
    }

}
