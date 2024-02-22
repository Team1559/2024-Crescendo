package frc.robot.constants;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;

public abstract class AbstractConstants {

    // ========================= Enums =========================================
    public static enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    private static enum RoboRioPortArrays {
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

        SwerveModuleHardwareIds(int driveMotorId, int steerMotorId, int cancoderId) {
            DRIVE_MOTOR_ID = driveMotorId;
            STEER_MOTOR_ID = steerMotorId;
            CANCODER_ID = cancoderId;
        }
    }

    // ========================= Static CONSTANTS ==============================
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = false;
    private static final AbstractConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final AbstractConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    public static final boolean TECHNICIAN_CONTROLLER_ENABLED = false;
    public static final AbstractConstants CONSTANTS = isGameRobot() ? GAME_ROBOT_CONSTANTS : TEST_ROBOT_CONSTANTS;

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

        System.out.println("XXXXXXXXXX " + id + " - " + canivoreId + " XXXXXXXXXX");
        canivoreId = canivoreId == null ? "" : canivoreId;

        Set<Integer> ids = uniqueCanBusIds.get(canivoreId);
        if (ids == null) {

            System.out.println("XXXXXXXXXX " + id + " - " + canivoreId + " XXXXXXXXXX - ids == null");
            uniqueCanBusIds.put(canivoreId, new HashSet<>(Arrays.asList(id)));
        } else if (!ids.add(id)) {
            System.out.println("XXXXXXXXXX " + id + " - " + canivoreId + " XXXXXXXXXX - else if");
            // TODO: Fix.
            // throw new RuntimeException("Duplicate ID (" + id + ") on " +
            // (canivoreId.isEmpty() ? "default" : canivoreId) + " CAN Bus!");
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
    public abstract Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets();

    private Map<WheelModuleIndex, SwerveModuleHardwareIds> swerveModuleHardwareIds = new HashMap<>(4) {
        {
            put(WheelModuleIndex.FRONT_LEFT,
                    new SwerveModuleHardwareIds(uniqueCanBusId(0, getCanivoreId()), uniqueCanBusId(1, getCanivoreId()),
                            uniqueCanBusId(2, getCanivoreId())));

            put(WheelModuleIndex.FRONT_RIGHT,
                    new SwerveModuleHardwareIds(uniqueCanBusId(3, getCanivoreId()), uniqueCanBusId(4, getCanivoreId()),
                            uniqueCanBusId(5, getCanivoreId())));

            put(WheelModuleIndex.BACK_LEFT,
                    new SwerveModuleHardwareIds(uniqueCanBusId(9, getCanivoreId()), uniqueCanBusId(10, getCanivoreId()),
                            uniqueCanBusId(11, getCanivoreId())));

            put(WheelModuleIndex.BACK_RIGHT,
                    new SwerveModuleHardwareIds(uniqueCanBusId(6, getCanivoreId()), uniqueCanBusId(7, getCanivoreId()),
                            uniqueCanBusId(8, getCanivoreId())));
        }
    };

    public Map<WheelModuleIndex, SwerveModuleHardwareIds> getSwerveModuleHardwareIds() {
        return swerveModuleHardwareIds;
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

    public final double SAFE_MOTOR_TEMPERATURE_BUFFER = 0.9;

    // #region: ----- Falcon 500 Motor -----

    /**
     * See: <a href=
     * "https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf">Falcon
     * 500 temperature test under maximum load conditions</a>
     * 
     * @return 109 degrees C
     */
    public Measure<Temperature> getFalcon500MaxTemperature() {
        return Celsius.of(109);
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
