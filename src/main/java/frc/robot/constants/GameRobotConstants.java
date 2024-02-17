package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

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
        return false;
    }

    @Override
    public boolean hasFlywheelSubsystem() {
        return false;
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
    // --- roboRIO ---
    @Override
    public String getRoboRioSerialNumber() {
        return "";
    }

    // --- Swerve ---
    @Override
    public Rotation2d[] getSwerveModuleEncoderOffsets() {
        return new Rotation2d[] {
                Rotation2d.fromRadians(0.120),
                Rotation2d.fromRadians(-0.023),
                Rotation2d.fromRadians(2.789),
                Rotation2d.fromRadians(0.853)
        };
    }
}
