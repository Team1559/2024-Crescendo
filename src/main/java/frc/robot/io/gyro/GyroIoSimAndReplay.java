package frc.robot.io.gyro;

/**
 * Provides no function.
 */
import frc.robot.Constants;

public class GyroIoSimAndReplay implements GyroIo {

    public GyroIoSimAndReplay() {
        // Nothing to define.
    }

    @Override
    public void updateInputs(GyroIoInputs inputs) {
        inputs.connected = true;
    }
}