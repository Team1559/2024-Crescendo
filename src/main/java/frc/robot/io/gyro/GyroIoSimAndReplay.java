package frc.robot.io.gyro;

/**
 * Provides no function.
 */
public class GyroIoSimAndReplay implements GyroIo {

    public GyroIoSimAndReplay() {
        // Nothing to define.
    }

    @Override
    public void updateInputs(GyroIoInputs inputs) {
        inputs.connected = true;
    }
}