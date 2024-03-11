package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.BetterLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.CommandUtils;

public class NoteSensor extends SubsystemBase {

    @AutoLog
    static class NoteSensorInputs {
        public boolean isObjectDetected;
    }

    private final DigitalInput limitSwitch;

    private NoteSensorInputsAutoLogged inputs = new NoteSensorInputsAutoLogged();

    public NoteSensor(int channel) {
        limitSwitch = new DigitalInput(channel);
    }

    @Override
    public void periodic() {
        inputs.isObjectDetected = isObjectDetectedOnSwitch();
        BetterLogger.processInputs("Shooter/NoteSensor", inputs);
    }

    /**
     * Check if limit switch is activated
     * 
     * @return limit switch state;
     */
    public boolean isObjectDetectedOnSwitch() {
        return !limitSwitch.get();
    }

    // ========================= Commands =========================
    public Command waitForObjectOnSwitchCommand() {
        return CommandUtils.addName(getName(), new WaitUntilCommand(this::isObjectDetectedOnSwitch));
    }

    public Command waitForNoObjectOnSwitchCommand() {
        return CommandUtils.addName(getName(), new WaitUntilCommand(() -> !isObjectDetectedOnSwitch()));
    }
}
