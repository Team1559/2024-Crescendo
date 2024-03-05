package frc.robot.io.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;

public class GyroIoPigeon2 implements GyroIo {

    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yawStatusSignal;
    private final StatusSignal<Double> yawVelocityStatusSignal;

    public GyroIoPigeon2(int deviceId, String canbus) {

        // Create Gyro.
        if (canbus == null) {
            pigeon = new Pigeon2(deviceId);
        } else {
            pigeon = new Pigeon2(deviceId, canbus);
        }

        // Config Gyro.
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        // Config StatusSignals for Logging.
        yawStatusSignal = pigeon.getYaw();
        yawVelocityStatusSignal = pigeon.getAngularVelocityZWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(100, yawStatusSignal, yawVelocityStatusSignal);

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIoInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yawStatusSignal, yawVelocityStatusSignal).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yawStatusSignal.getValueAsDouble()).plus(Rotation2d.fromDegrees(0));
        inputs.yawVelocity = DegreesPerSecond.of(yawVelocityStatusSignal.getValueAsDouble());
    }
}