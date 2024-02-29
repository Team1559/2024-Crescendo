package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;

public class GameRobotConstants extends AbstractConstants {

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
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
    public Measure<Distance> getClimberMaxHeight() {
        return Inches.of(12);
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
        return new PID(.6, 0, 0);
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
    public boolean isFeederMotorInverted() {
        return true;
    }

    @Override
    public PID getFeederPidValues() {
        return new PID(0.33 / CONSTANTS.getFeederVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    @Override
    public Measure<Velocity<Angle>> getFeederVelocityForward() {
        // TODO: Configure Value.
        return RevolutionsPerSecond.of(3666);
    }

    @Override
    public Measure<Velocity<Angle>> getFeederVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
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
    public double flywheelSpinOffset() {
        // TODO: Tune.
        return 1;
    }

    // #endregion

    // #region: ----- Intake -----
    @Override
    public boolean isIntakeMotorInverted() {
        return true;
    }

    @Override
    public PID getIntakePidValues() {
        return new PID(0.33 / CONSTANTS.getIntakeVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    @Override
    public Measure<Velocity<Angle>> getIntakeVelocityForward() {
        // TODO: Configure Value.
        return RevolutionsPerSecond.of(8250);
    }

    @Override
    public Measure<Velocity<Angle>> getIntakeVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
    }

    // #endregion

    // #region: ----- LEDs -----
    @Override
    public int getLedLength() {
        return 144 * 3; // This is the number that WILL be on the robot
    }

    // #endregion

    // #region: -------- roboRIO --------
    @Override
    public String getRoboRioSerialNumber() {
        return "03282B9F";
    }

    // #endregion

    // #region: -------- Swerve --------
    @Override
    public Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets() {
        return new HashMap<>() {
            {
                put(WheelModuleIndex.FRONT_LEFT, Rotation2d.fromRadians(0.120));
                put(WheelModuleIndex.FRONT_RIGHT, Rotation2d.fromRadians(-0.023));
                put(WheelModuleIndex.BACK_LEFT, Rotation2d.fromRadians(2.789));
                put(WheelModuleIndex.BACK_RIGHT, Rotation2d.fromRadians(0.853));
            }
        };
    }

    // #endregion

    // #region: -------- Traverser --------

    @Override
    public boolean isTraverserInverted() {
        return true;
    }

    @Override
    public PID getTraverserPidValues() {
        return new PID(0.33 / CONSTANTS.getTraverserVelocity().in(RevolutionsPerSecond), 0, 0, 11.0 / 11000);
    }

    @Override
    public Measure<Velocity<Angle>> getTraverserVelocity() { // TODO
        return RevolutionsPerSecond.of(5000);
    }

    // #endregion

    // #region: ----- Vision -----
    @Override
    public String getCameraNameBack() {
        return "limelight-back";
    }

    @Override
    public String getCameraNameFront() {
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
