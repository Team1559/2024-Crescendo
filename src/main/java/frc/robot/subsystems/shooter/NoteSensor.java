package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class NoteSensor extends SubsystemBase {

    @AutoLog
    static class NoteSensorInputs {
        public boolean isObjectDetectedSwitch;
    }

    private NoteSensorInputsAutoLogged inputs = new NoteSensorInputsAutoLogged();
    private final DigitalInput limitSwitch;

    public NoteSensor(I2C.Port port, int channel) {
        limitSwitch = new DigitalInput(channel);
    }

    public NoteSensor() {
        this(I2C.Port.kOnboard, 2);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/NoteSensor", inputs);
    }

    private void updateInputs() {
        inputs.isObjectDetectedSwitch = !limitSwitch.get();
    }

    /**
     * Check if limit switch is activated
     * 
     * @return limit switch state;
     */
    public boolean isObjectDetected() {
        return inputs.isObjectDetectedSwitch;
    }

    // ========================= Commands =========================
    public Command waitForObjectCommandSwitch() {
        return new WaitUntilCommand(this::isObjectDetected);
    }

    public Command waitForNoObjectCommandSwitch() {
        return new WaitUntilCommand(() -> !isObjectDetected());
    }
}
