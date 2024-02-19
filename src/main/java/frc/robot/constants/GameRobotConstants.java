package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

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

    // #region: --------------- Driving Configurations -------------------------
    public Measure<Velocity<Angle>> getMaxAngularSpeed() {
        // TODO: Tune.
        return Radians.of(getMaxLinearSpeed().in(MetersPerSecond) / CONSTANTS.getWheelRadius().in(Meters)).per(Second);
    }

    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
        // TODO: Tune.
        return MetersPerSecond.of(3);
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------
    // #region: ----- Aimer -----
    @Override
    public int getAimerMotorIdLeft() {
        return 23;
    }

    @Override
    public int getAimerMotorIdRight() {
        return 22;
    }

    // #endregion

    // #region: ----- Feeder -----
    @Override
    public int getFeederMotorId() {
        return 21;
    }

    @Override
    public boolean isFeederMortorInverted() {
        return true;
    }

    // #endregion

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

    // #region: ----- Vision -----

    @Override
    public String getCameraName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCameraNameBack'");
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
    public double getGearRatioOfDriveWheel() {
        // L3 Gear ratio.
        return (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    }

    @Override
    public double getGearRatioOfTurnWheel() {
        return 12.8;
    }

    @Override
    public Measure<Distance> getWheelDistanceFrontToBack() {
        return Inches.of(24); // TODO: Measure.
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(24); // TODO: Measure.
    }

    public Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }
    // #endregion
}
