package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class NoteSensor extends SubsystemBase {

    @AutoLog
    static class NoteSensorInputs {
        public boolean isObjectDetectedOnSwitch;
    }

    private final DigitalInput limitSwitch;

    private NoteSensorInputsAutoLogged inputs = new NoteSensorInputsAutoLogged();

    public NoteSensor(int channel) {
        limitSwitch = new DigitalInput(channel);
    }

    @Override
    public void periodic() {
        inputs.isObjectDetectedOnSwitch = isObjectDetectedOnSwitch();
        Logger.processInputs("Shooter/NoteSensor", inputs);
    }

    /**
     * Check if limit switch is activated
     * 
     * @return limit switch state;
     */
    public boolean isObjectDetectedOnSwitch() {
        return !limitSwitch.get();
    }

    public boolean isObjectNotDetectedSwitch() {
        return !inputs.isObjectDetectedOnSwitch;
    }

    // ========================= Commands =========================
    public Command waitForObjectOnSwitchCommand() {
        return new WaitUntilCommand(this::isObjectDetectedOnSwitch);
    }

    public Command waitForNoObjectOnSwitchCommand() {
        return new WaitUntilCommand(() -> !isObjectDetectedOnSwitch());
    }
}
