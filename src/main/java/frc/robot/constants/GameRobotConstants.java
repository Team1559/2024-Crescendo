package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.opencv.core.Mat.Tuple2;

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
        return true;
    }

    @Override
    public boolean hasNoteSensorSubsystem() {
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
    public boolean hasVisionSubsystem() {
        return true;
    }

    @Override
    public boolean hasTraverserSubsystem() {
        return false;
    }

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public Measure<Velocity<Angle>> getMaxAngularSpeed() {
        // TODO: Tune.
        return DegreesPerSecond.of(360);
    }

    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
        // TODO: Tune.
        return MetersPerSecond.of(5);
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------

    // #region: ----- Climber --------
    @Override
    public PID getClimberPid() {
        return new PID(.1, 0, 0);
    }

    @Override
    public double getClimberMaxHeight() {
        return 12;
    }

    // #endregion

    // #region: ----- Aimer -----
    @Override
    public Tuple2<Rotation2d> getAimerAngleRange() {
        return new Tuple2<Rotation2d>(Rotation2d.fromDegrees(1), Rotation2d.fromDegrees(40));
    }

    @Override
    public Rotation2d getAimerEncoderOffset() {
        return Rotation2d.fromRadians(2.599);
    }

    @Override
    public PID getAimerPid() {
        return new PID(0.5, .0, 0);
    }

    // #endregion

    // #region: ----- Color Sensor -----
    @Override
    public int getColorSensorProximityThreshold() {
        return 200; // TODO: Configure Value.
    }

    // #endregion

    // #region: ----- Feeder -----
    @Override
    public boolean isFeederMortorInverted() {
        return true;
    }

    @Override
    public double getFeederForwardVoltage() {
        // TODO: Configure Value.
        return 4;
    }

    @Override
    public double getFeederReverseVoltage() {
        // TODO: Configure Value.
        return -getFeederForwardVoltage();
    }

    // #endregion

    // #region: ----- Flywheel -----
    @Override
    public double getFlywheelForwardVoltage() {
        // TODO: Configure Value.
        return 10;
    }

    @Override
    public double getFlywheelReverseVoltage() {
        // TODO: Configure Value.
        return -6;
    }

    @Override
    public double getFlywheelMotorPowerDifferentialPercentage() {
        // TODO: Tune.
        return 0.75;
    }

    // #endregion

    // #region: ----- Intake -----
    @Override
    public boolean isIntakeMortorInverted() {
        return true;
    }

    @Override
    public double getIntakeForwardVoltage() {
        // TODO: Configure Value.
        return 9;
    }

    @Override
    public double getIntakeReverseVoltage() {
        // TODO: Configure Value.
        return -getFeederForwardVoltage();
    }

    // #endregion

    // #region: ----- LEDs -----
    @Override
    public int getLedLenth() {
        return 144 * 3; // This is the number that WILL be on the robot
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
    public boolean isTraverserInverted() {
        return true;
    }

    // #endregion

    // #region: ----- Vision -----

    @Override
    public String getCameraName() {
        // TODO Auto-generated method stub
        return "limelight";
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
        return Inches.of(23);
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(23);
    }

    public Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }
    // #endregion
}
