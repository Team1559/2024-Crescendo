package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class GameRobotConstants extends AbstractConstants {

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
    // #region: --------------- Alliance ---------------------------------------
    public boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
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

    // #endregion

    // #region: --------------- Hardware ---------------------------------------
    // #region: -------- roboRIO --------
    @Override
    public String getRoboRioSerialNumber() {
        return ""; // TODO.
    }

    // #endregion

    // #region: -------- Swerve --------
    @Override
    public Rotation2d[] getSwerveModuleEncoderOffsets() {
        return new Rotation2d[] {
                Rotation2d.fromRadians(0.120),
                Rotation2d.fromRadians(-0.023),
                Rotation2d.fromRadians(2.789),
                Rotation2d.fromRadians(0.853)
        };
    }

    // #endregion

    // #region: -------- Traverser --------
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

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return true;
    }

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    @Override
    public Measure<Distance> getWheelDistanceFrontToBack() {
        return Inches.of(24); // TODO: Measure.
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(24); // TODO: Measure.
    }

    // #endregion
}
