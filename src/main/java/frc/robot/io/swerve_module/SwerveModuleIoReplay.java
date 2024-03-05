package frc.robot.io.swerve_module;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Voltage;

/**
 * No functionality provided because none needed for replay.
 */
import frc.robot.Constants;

public class SwerveModuleIoReplay implements SwerveModuleIo {

    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        // No functionality.
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> volts) {
        // No functionality.
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {
        // No functionality.
    }
}