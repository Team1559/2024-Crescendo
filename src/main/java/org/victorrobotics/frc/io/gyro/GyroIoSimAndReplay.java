package org.victorrobotics.frc.io.gyro;

public class GyroIoSimAndReplay implements GyroIo {

    public GyroIoSimAndReplay() {
        // Nothing to define.
    }

    @Override
    public void updateInputs(GyroIoInputs inputs) {
        inputs.connected = true;
    }
}