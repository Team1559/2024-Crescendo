package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class GameRobotConstants extends AbstractConstants {

    // ========================= CONSTANTS ======================================
    // ---------- Operation Modes ----------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return true;
    }

    // ---------- Alliance ----------
    public boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // ---------- Capabilities Flags --------
    @Override
    public boolean hasAimerSubsystem() {
        return true;
    }

    @Override
    public boolean hasClimberSubsystem() {
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

    @Override
    public boolean hasTraverserSubsystem() {
        return false;
    }

    // ---------- Hardware ----------
    // --- roboRIO ---
    @Override
    public String getRoboRioSerialNumber() {
        return ""; // TODO.
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

    // --- Traverser ---
    @Override
    public double getTraverserFowardVoltage() {
        return 6.0;
    }

    @Override
    public double getTraverserReverseVoltage() {
        return -getTraverserFowardVoltage();
    }

    @Override
    public int getTraverserMotorId() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Traverser");
    }

    @Override
    public boolean isTraverserInverted() {
        return true;
    }

}
