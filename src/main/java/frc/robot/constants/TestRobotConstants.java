package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class TestRobotConstants extends AbstractConstants {

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
    // #region: --------------- Alliance ---------------------------------------
    public boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
    @Override
    public boolean hasAimerSubsystem() {
        return false;
    }

    @Override
    public boolean hasClimberSubsystem() {
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
    public boolean hasTraverserSubsystem() {
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

    // #endregion

    // #region: --------------- Hardware ---------------------------------------
    // #region: ----- roboRIO -----
    @Override
    public String getRoboRioSerialNumber() {
        return "03282BB6";
    }

    // #endregion

    // #region: ----- Swerve -----
    @Override
    public Rotation2d[] getSwerveModuleEncoderOffsets() {
        return new Rotation2d[] {
                Rotation2d.fromRotations(-0.300293),
                Rotation2d.fromRotations(-0.228760),
                Rotation2d.fromRotations(-0.238525),
                Rotation2d.fromRotations(-0.000732)
        };
    }

    // #endregion

    // #region: ----- Traverser -----
    @Override
    public double getTraverserFowardVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getTraverserFowardVoltage'");
    }

    @Override
    public double getTraverserReverseVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getTraverserReverseVoltage'");
    }

    @Override
    public int getTraverserMotorId() {
        throw new UnsupportedOperationException("Unimplemented method 'getTraverserMotorId'");
    }

    @Override
    public boolean isTraverserInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'isTraverserInverted'");
    }

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return false;
    }

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    @Override
    public Measure<Distance> getWheelDistanceFrontToBack() {
        return Inches.of(24);
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(24);
    }

    public Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }

    // #endregion
}
