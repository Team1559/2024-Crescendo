package frc.robot.constants;

public class TestRobotConstants extends AbstractConstants {

    // ========================= CONSTANTS ======================================
    // ---------- Operation Modes ----------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return false;
    }

    // ---------- Capabilities Flags --------
    @Override
    public boolean hasAimerSubsystem() {
        return false;
    }

    @Override
    public boolean hasColorSensorSubsystem() {
        return false;
    }

    @Override
    public boolean hasFeederSubsystem() {
        return false;
    }

    @Override
    public boolean hasFlywheelSubsystem() {
        return false;
    }

    @Override
    public boolean hasIntakeSubsystem() {
        return false;
    }

    @Override
    public boolean hasShooterSubsystemGroup() {
        return hasAimerSubsystem() && hasColorSensorSubsystem() && hasFeederSubsystem() && hasFlywheelSubsystem()
                && hasIntakeSubsystem();
    }

    @Override
    public boolean hasVisionSubsystem() {
        return true;
    }

    // ---------- Hardware ----------
    @Override
    public String getRoboRioSerialNumber() {
        return "03282BB6";
    }
}
