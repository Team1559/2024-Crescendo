package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public abstract class AbstractConstants {

    // ========================= Enums =========================================
    public static enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    // ========================= Static Classes ================================
    public static class PID {
        public final double P, I, D;

        PID(double p, double i, double d) {
            P = p;
            I = i;
            D = d;
        }
    }

    public static class SwirveModuleHardwareIds {
        public final int DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID;

        SwirveModuleHardwareIds(int driveMotorId, int steerMotorId, int cancoderId) {
            DRIVE_MOTOR_ID = driveMotorId;
            STEER_MOTOR_ID = steerMotorId;
            CANCODER_ID = cancoderId;
        }
    }

    // ========================= Static CONSTANTS ==============================
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = false;
    private static final AbstractConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final AbstractConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    public static final AbstractConstants CONSTANTS = isGameRobot() ? GAME_ROBOT_CONSTANTS : TEST_ROBOT_CONSTANTS;

    // ========================= Static Methods ================================
    public static boolean isGameRobot() {

        String roboRioSerialNumber = System.getenv("serialnum");
        roboRioSerialNumber = roboRioSerialNumber == null ? "" : roboRioSerialNumber.trim();

        return FORCE_GAME_ROBOT_CONSTANTS
                || roboRioSerialNumber.equalsIgnoreCase(GAME_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || !roboRioSerialNumber.equalsIgnoreCase(TEST_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || DriverStation.getMatchType() != MatchType.None
                || DriverStation.getMatchNumber() != 0
                || !Objects.equals(DriverStation.getEventName(), "");
    }

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
    // #region: --------------- Alliance ---------------------------------------
    /**
     * @return The assigned Alliance or the {@link #getDefaultAllianceForAuto}, if
     *         no alliance is set.
     */
    public Alliance getAssignedAlliance() {
        return DriverStation.getAlliance().orElse(getDefaultAllianceForAuto());
    };

    /**
     * @return The side of the field that the automation paths are made for.
     */
    public Alliance getDefaultAllianceForAuto() {
        return Alliance.Blue;
    }

    public abstract boolean shouldFlipPathIfAssignedAllianceIsNotDefault();

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
    public abstract boolean hasAimerSubsystem();

    public abstract boolean hasClimberSubsystem();

    public abstract boolean hasColorSensorSubsystem();

    public abstract boolean hasFeederSubsystem();

    public abstract boolean hasFlywheelSubsystem();

    public abstract boolean hasIntakeSubsystem();

    public abstract boolean hasShooterSubsystemGroup();

    public abstract boolean hasTraverserSubsystem();

    public abstract boolean hasVisionSubsystem();

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public double getJoystickDeadband() {
        return 0.2;
    }

    public abstract Measure<Velocity<Angle>> getMaxAngularSpeed();

    public abstract Measure<Velocity<Distance>> getMaxLinearSpeed();
    // #endregion

    // #region: --------------- Game Objects -----------------------------------
    public Translation2d getSpeakerLocation() {
        if (getAssignedAlliance() == Alliance.Blue) {
            return new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));
        } else {
            return new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));
        }
    }

    public Translation2d getAmpLocation() {
        if (getAssignedAlliance() == Alliance.Blue) {
            return new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00));
        } else {
            return new Translation2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00));
        }
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------
    // #region: ----- Aimer -----
    public abstract Tuple2<Rotation2d> getAimerAngleRange();

    public abstract Rotation2d getAimerEncoderOffset();

    public abstract int getAimerEncoderPort();

    public abstract int getAimerMotorIdLeft();

    public abstract int getAimerMotorIdRight();

    public abstract PID getAimerPid();

    // #endregion

    // #region: ----- Canivore -----
    public String getCanivoreBusId() {
        return "1559Canivore";
    }

    // #endregion

    // #region: ----- Color Sensor -----
    public abstract int getColorSensorProximityThreshold(); // 1500; // TODO: Configure Value.

    // #endregion

    // #region: ----- Feeder -----
    public abstract int getFeederMotorId();

    public abstract boolean isFeederMortorInverted();

    public abstract double getFeederForwardVoltage();

    public abstract double getFeederReverseVoltage();

    // #endregion

    // #region: ----- Flywheel -----
    public abstract int getFlywheelMotorIdLeft();

    public abstract int getFlywheelMotorIdRight();

    public abstract double getFlywheelForwardVoltage();

    public abstract double getFlywheelReverseVoltage();

    // #endregion

    // #region: ----- Gyro -----
    public int getGyroId() {
        return 12;
    }

    // #endregion

    // #region: ----- Intake -----
    public abstract int getIntakeMotorId();

    public abstract boolean isIntakeMortorInverted();

    public abstract double getIntakeForwardVoltage();

    public abstract double getIntakeReverseVoltage();

    // #endregion

    // #region: ----- LEDs -----
    public int getLedPort() {
        return 0;
    }

    public abstract int getLedLenth(); // 144

    // #endregion

    // #region: ----- roboRIO -----
    public abstract String getRoboRioSerialNumber();

    // #endregion

    // #region: ----- Swerve --------
    /**
     * The index of the Rotation matches the Index of the Module in Advaltage Scope.
     * <p>
     * Note: Offsetting by 180 degrees will invert the direction the wheel spins.
     * </p>
     */
    public abstract Rotation2d[] getSwerveModuleEncoderOffsets();

    public SwirveModuleHardwareIds getSwirveModuleHardwareIdsFrontLeft() {
        return new SwirveModuleHardwareIds(0, 1, 2);
    }

    public SwirveModuleHardwareIds getSwirveModuleHardwareIdsFrontRight() {
        return new SwirveModuleHardwareIds(3, 4, 5);
    }

    public SwirveModuleHardwareIds getSwirveModuleHardwareIdsBackLeft() {
        return new SwirveModuleHardwareIds(9, 10, 11);
    }

    public SwirveModuleHardwareIds getSwirveModuleHardwareIdsBackRight() {
        return new SwirveModuleHardwareIds(6, 7, 8);
    }

    // #endregion

    // #region: ----- Traverser -----
    public abstract double getTraverserFowardVoltage();

    public abstract double getTraverserReverseVoltage();

    public abstract int getTraverserMotorId();

    public abstract boolean isTraverserInverted();

    // #endregion

    // #region: ----- Vision -----
    public abstract String getCameraName();

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    public Measure<Distance> getDriveBaseWheelRadius() {
        return Meters.of(Math.hypot(getWheelDistanceFrontToBack().divide(2).in(Meters),
                getWheelDistanceLeftToRight().divide(2).in(Meters)));
    }

    public abstract double getGearRatioOfDriveWheel();

    public abstract double getGearRatioOfTurnWheel();

    /**
     * @return The distance between the middle of the front wheel to middle of the
     *         back wheel (X coordinates).
     */
    public abstract Measure<Distance> getWheelDistanceFrontToBack();

    /**
     * @return The distance between the middle of the left wheel to middle of the
     *         right wheel (y coordinates).
     */
    public abstract Measure<Distance> getWheelDistanceLeftToRight();

    public abstract Measure<Distance> getWheelRadius();
    // #endregion
}
