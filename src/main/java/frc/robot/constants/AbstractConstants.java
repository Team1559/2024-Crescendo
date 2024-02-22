package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class AbstractConstants {

    // ========================= Enums =========================================
    public enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    private enum RoboRioPortArrays {
        DIO,
        PWM
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

    public static class SwerveModuleHardwareIds {

        public final int DRIVE_MOTOR_ID, STEER_MOTOR_ID, CANCODER_ID;

        /**
         * Saves these IDs for reference, and ensured uniqueness on the default CAN BUS.
         */
        SwerveModuleHardwareIds(int driveMotorId, int steerMotorId, int cancoderId) {
            this(driveMotorId, steerMotorId, cancoderId, null);
        }

        /**
         * Saves these IDs for reference, and ensured uniqueness on the CAN BUS with the
         * given Canivore Id.
         */
        SwerveModuleHardwareIds(int driveMotorId, int steerMotorId, int cancoderId, String canivoreId) {
            DRIVE_MOTOR_ID = uniqueCanBusId(driveMotorId, canivoreId);
            STEER_MOTOR_ID = uniqueCanBusId(steerMotorId, canivoreId);
            CANCODER_ID = uniqueCanBusId(cancoderId, canivoreId);
        }
    }

    // ========================= Static CONSTANTS ==============================
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = false;
    private static final AbstractConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final AbstractConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    public static final boolean TECHNICIAN_CONTROLLER_ENABLED = false;
    public static final AbstractConstants CONSTANTS = isGameRobot() ? GAME_ROBOT_CONSTANTS : TEST_ROBOT_CONSTANTS;

    private static final Translation3d SPEKER_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));
    private static final Translation3d SPEKER_LOCATION_RED = new Translation3d(Units.inchesToMeters(652.73),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));
    private static final Translation3d AMP_LOCATION_RED = new Translation3d(Units.inchesToMeters(72.5),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));
    private static final Translation3d AMP_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(578.77),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));
    // ========================= Static Variables ==============================
    private static final Map<String, Set<Integer>> uniqueCanBusIds = new HashMap<>();
    private static final Map<RoboRioPortArrays, Set<Integer>> uniqueRoboRioPorts = new HashMap();

    // ========================= Static Methods ================================
    public static boolean isGameRobot() {

        String roboRioSerialNumber = System.getenv("serialnum");
        roboRioSerialNumber = roboRioSerialNumber == null ? "" : roboRioSerialNumber.trim();

        return FORCE_GAME_ROBOT_CONSTANTS
                || roboRioSerialNumber.equalsIgnoreCase(GAME_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || !roboRioSerialNumber.equalsIgnoreCase(TEST_ROBOT_CONSTANTS.getRoboRioSerialNumber());
    }

    private static int uniqueCanBusId(int id) {
        return uniqueCanBusId(id, null);
    }

    private static int uniqueCanBusId(int id, String canivoreId) {
        System.out.println(id + " - " + canivoreId);
        canivoreId = canivoreId == null ? "" : canivoreId;

        Set<Integer> ids = uniqueCanBusIds.get(canivoreId);
        if (ids == null) {
            uniqueCanBusIds.put(canivoreId, new HashSet<>(Arrays.asList(id)));
        } else if (!ids.add(id)) {
            // TODO: Fix.
            // throw new RuntimeException(
            // "Duplicate ID (" + id + ") on " + (canivoreId.isEmpty() ? "default" :
            // canivoreId) + " CAN Bus!");
        }

        return id;
    }

    private static int uniqueRoboRioPort(int port, RoboRioPortArrays portArray) {

        Set<Integer> ports = uniqueRoboRioPorts.get(portArray);
        if (ports == null) {
            ports = new HashSet<>() {
                {
                    add(port);
                }
            };
            uniqueRoboRioPorts.put(portArray, ports);
        } else if (!ports.add(port)) {
            throw new RuntimeException("Duplicate roboRIO Port (" + port + ")!");
        }

        return port;
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

    public abstract boolean hasTraverserSubsystem();

    public abstract boolean hasVisionSubsystem();

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public double getJoystickDeadband() {
        return 0.05;
    }

    public abstract Measure<Velocity<Angle>> getMaxAngularSpeed();

    public abstract Measure<Velocity<Distance>> getMaxLinearSpeed();
    // #endregion

    // #region: --------------- Game Objects -----------------------------------
    public Translation3d getSpeakerLocation() {
        if (getAssignedAlliance() == Alliance.Blue) {
            return CONSTANTS.SPEKER_LOCATION_BLUE;
        } else {
            return CONSTANTS.SPEKER_LOCATION_RED;
        }
    }

    public Translation3d getAmpLocation() {
        if (getAssignedAlliance() == Alliance.Blue) {
            return CONSTANTS.AMP_LOCATION_RED;
        } else {
            return CONSTANTS.AMP_LOCATION_BLUE;
        }
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------

    // #region: ----- Climber --------
    public int getClimberMotorIdLeft() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Left Climber motor");
    }

    public int getClimberMotorIdRight() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Right Climber motor");
    }

    public abstract PID getClimberPid();

    public abstract double getClimberMaxHeight();
    // #endregion

    // #region: ----- Aimer -----
    public abstract Tuple2<Rotation2d> getAimerAngleRange();

    public abstract Rotation2d getAimerEncoderOffset();

    public int getAimerEncoderPort() {
        return uniqueRoboRioPort(0, RoboRioPortArrays.DIO);
    }

    public int getAimerMotorIdLeft() {
        return uniqueCanBusId(23, getCanivoreId());
    }

    public int getAimerMotorIdRight() {
        return uniqueCanBusId(22, getCanivoreId());
    }

    public abstract PID getAimerPid();

    // #endregion

    // #region: ----- Canivore -----
    public String getCanivoreId() {
        return "1559Canivore";
    }

    // #endregion

    // #region: ----- Color Sensor -----
    public abstract int getColorSensorProximityThreshold();

    // #endregion

    // #region: ----- Feeder -----
    public int getFeederMotorId() {
        return uniqueCanBusId(21, getCanivoreId());
    }

    public abstract boolean isFeederMortorInverted();

    public abstract double getFeederForwardVoltage();

    public abstract double getFeederReverseVoltage();

    // #endregion

    // #region: ----- Flywheel -----
    public int getFlywheelMotorIdLeft() {
        return uniqueCanBusId(24, getCanivoreId());
    }

    public int getFlywheelMotorIdRight() {
        return uniqueCanBusId(25, getCanivoreId());
    }

    public abstract double getFlywheelForwardVoltage();

    public abstract double getFlywheelReverseVoltage();

    public abstract double getFlywheelMotorPowerDifferentialPercentage();

    // #endregion

    // #region: ----- Gyro -----
    public int getGyroId() {
        return uniqueCanBusId(12, getCanivoreId());
    }

    // #endregion

    // #region: ----- Intake -----
    public int getIntakeMotorId() {
        return uniqueCanBusId(20, getCanivoreId());
    }

    public abstract boolean isIntakeMortorInverted();

    public abstract double getIntakeForwardVoltage();

    public abstract double getIntakeReverseVoltage();

    // #endregion

    // #region: ----- LEDs -----
    public int getLedPort() {
        return uniqueRoboRioPort(0, RoboRioPortArrays.PWM);
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

    public SwerveModuleHardwareIds getSwerveModuleHardwareIdsFrontLeft() {
        return new SwerveModuleHardwareIds(0, 1, 2);
    }

    public SwerveModuleHardwareIds getSwerveModuleHardwareIdsFrontRight() {
        return new SwerveModuleHardwareIds(3, 4, 5);
    }

    public SwerveModuleHardwareIds getSwerveModuleHardwareIdsBackLeft() {
        return new SwerveModuleHardwareIds(9, 10, 11);
    }

    public SwerveModuleHardwareIds getSwerveModuleHardwareIdsBackRight() {
        return new SwerveModuleHardwareIds(6, 7, 8);
    }

    // #endregion

    // #region: ----- Traverser -----
    public abstract double getTraverserFowardVoltage();

    public abstract double getTraverserReverseVoltage();

    public int getTraverserMotorId() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Traverser");
    }

    public abstract boolean isTraverserInverted();

    // #endregion

    // #region: ----- Vision -----
    public abstract String getCameraName();

    // #endregion

    // #region: --------------- Motor / Motor Controller Settings --------------

    // #region: ----- Falcon 500 Motor -----

    /**
     * Allow 40A continuous, 80A momentary supply current. See: <a href=
     * "https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html">CTR
     * Electronics: CurrentLimitsConfigs</a>
     * 
     * @return A {@link CurrentLimitsConfigs} object with the default Current
     *         limits.
     */
    public CurrentLimitsConfigs getFalcon500CurrentLimitsConfigs() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40.0;
        limits.SupplyCurrentThreshold = 80.0;
        limits.SupplyTimeThreshold = 0.5;
        return limits;
    }

    // #endregion

    // #region: ----- NEO 550 Brushless Motor -----

    /**
     * @return Value as Amps.
     */
    public int getNeo550BrushlessCurrentLimit() {
        return 24;
    }

    /**
     * @return Value as Amps.
     */
    public int getNeo550BrushlessCurrentSecondaryLimit() {
        return 80;
    }

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();

    // #endregion

    // #region: --------------- PathPlanner Settings ---------------------------
    /**
     * @return Value in Times per Second.
     */
    public double getPathPlannerLogUpdateFrequencyDefault() {
        return 50;
    }

    /**
     * This higher rate is needed by PathPlanner.
     * 
     * @return Value in Times per Second.
     */
    public double getPathPlannerLogFrequencyForOdometry() {
        return 100;
    }

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
