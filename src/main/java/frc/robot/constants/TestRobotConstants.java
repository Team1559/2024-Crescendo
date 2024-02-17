package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

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
    // --- roboRIO ---
    @Override
    public String getRoboRioSerialNumber() {
        return "03282BB6";
    }

    // --- Swerve ---
    @Override
    public Rotation2d[] getSwerveModuleEncoderOffsets() {
        return new Rotation2d[] {
                Rotation2d.fromRotations(-0.300293),
                Rotation2d.fromRotations(-0.228760),
                Rotation2d.fromRotations(-0.238525),
                Rotation2d.fromRotations(-0.000732)
        };
    }
}
