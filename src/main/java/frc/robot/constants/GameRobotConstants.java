package frc.robot.constants;

public class GameRobotConstants extends AbstractConstants {

    // ========================= CONSTANTS ======================================
    // ---------- Operation Modes ----------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return true;
    }

    // ---------- Capabilities Flags --------
    @Override
    public boolean hasAimerSubsystem() {
        return false;
    }

    @Override
    public boolean hasColorSensorSubsystem() {
        return true;
    }

    @Override
    public boolean hasFeederSubsystem() {
        return true;
    }

    @Override
    public boolean hasFlywheelSubsystem() {
        return true;
    }

    @Override
    public boolean hasIntakeSubsystem() {
        return true;
    }

    @Override
    public boolean hasShooterSubsystemGroup() {
        return hasAimerSubsystem() && hasColorSensorSubsystem() && hasFeederSubsystem() && hasFlywheelSubsystem()
                && hasIntakeSubsystem();
    }

    @Override
    public boolean hasVisionSubsystem() {
        return false;
    }

    // ---------- Hardware ----------
    @Override
    public String getRoboRioSerialNumber() {
        return "";
    }
}
