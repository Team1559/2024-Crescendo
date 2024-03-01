package frc.robot.constants;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve_module.IndexedSwerveModule.WheelModuleIndex;

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
    /**
     * Contains PID values and a Feed Forward (FF) value.
     */
    public static class PID {

        public final double P, I, D, FF;

        /**
         * Constructs this PID object with a FF value of 0.
         */
        public PID(double p, double i, double d) {
            this(p, i, d, 0);
        }

        public PID(double p, double i, double d, double ff) {
            P = p;
            I = i;
            D = d;
            FF = ff;
        }

        /**
         * @returns A {@link PIDController} with the defined {@link #P}, {@link #I}, &
         *          {@link #D} values.
         */
        public PIDController createController() {
            return new PIDController(P, I, D);
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

    // ========================= Static Variables ==============================
    private static Map<String, Set<Integer>> uniqueCanBusIds;
    private static Map<RoboRioPortArrays, Set<Integer>> uniqueRoboRioPorts;

    // ========================= Static Methods ================================
    private static int uniqueCanBusId(int id) {
        return uniqueCanBusId(id, null);
    }

    private static int uniqueCanBusId(int id, String canivoreId) {

        uniqueCanBusIds = uniqueCanBusIds == null ? new HashMap<>() : uniqueCanBusIds;

        canivoreId = canivoreId == null ? "" : canivoreId;
        Set<Integer> ids = uniqueCanBusIds.get(canivoreId);
        if (ids == null) {
            uniqueCanBusIds.put(canivoreId, new HashSet<>(Arrays.asList(id)));
        } else if (!ids.add(id)) {
            throw new RuntimeException(
                    "Duplicate ID (" + id + ") on " + (canivoreId.isEmpty() ? "default" : canivoreId) + " CAN Bus!");
        }

        return id;
    }

    private static int uniqueRoboRioPort(int port, RoboRioPortArrays portArray) {

        uniqueRoboRioPorts = uniqueRoboRioPorts == null ? new HashMap<>() : uniqueRoboRioPorts;
        Set<Integer> ports = uniqueRoboRioPorts.get(portArray);
        if (ports == null) {
            uniqueRoboRioPorts.put(portArray, new HashSet<>(Arrays.asList(port)));
        } else if (!ports.add(port)) {
            throw new RuntimeException("Duplicate roboRIO Port (" + port + ")!");
        }

        return port;
    }

    // ==================== "CONSTANTS" (Ctrl + K, Ctrl + 8 to fold regions) ===
    // #region: --------------- Game Robot / Technician ------------------------
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = true;
    public static final boolean TECHNICIAN_CONTROLLER_ENABLED = false && !FORCE_GAME_ROBOT_CONSTANTS;

    private static final AbstractConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final AbstractConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();
    public static final AbstractConstants CONSTANTS = isGameRobot() ? GAME_ROBOT_CONSTANTS : TEST_ROBOT_CONSTANTS;

    private static boolean isGameRobot() {
        String roboRioSerialNumber = System.getenv("serialnum");
        System.out.println("Serial Number = " + System.getenv("serialnum"));
        roboRioSerialNumber = roboRioSerialNumber == null ? "" : roboRioSerialNumber.trim();

        return FORCE_GAME_ROBOT_CONSTANTS
                || roboRioSerialNumber.equalsIgnoreCase(GAME_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || !roboRioSerialNumber.equalsIgnoreCase(TEST_ROBOT_CONSTANTS.getRoboRioSerialNumber());
    }

    // #endregion

    // #region: --------------- Alliance ---------------------------------------
    /**
     * @return The assigned Alliance or the {@link #getDefaultAllianceForAuto}, if
     *         no alliance is set.
     */
    public Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(getDefaultAllianceForAuto());
    };

    /**
     * @return The side of the field that the automation paths are made for.
     */
    public Alliance getDefaultAllianceForAuto() {
        return Alliance.Blue;
    }

    public boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
    public abstract boolean hasAimerSubsystem();

    public abstract boolean hasClimberSubsystem();

    public abstract boolean hasNoteSensorSubsystem();

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
    private static final Translation3d SPEAKER_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));
    private static final Translation3d SPEAKER_LOCATION_RED = new Translation3d(Units.inchesToMeters(652.73),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));

    public Translation3d getSpeakerLocation() {
        if (getAlliance() == Alliance.Blue) {
            return AbstractConstants.SPEAKER_LOCATION_BLUE;
        } else {
            return AbstractConstants.SPEAKER_LOCATION_RED;
        }
    }

    private static final Translation3d AMP_LOCATION_RED = new Translation3d(Units.inchesToMeters(72.5),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));
    private static final Translation3d AMP_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(578.77),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));

    public Translation3d getAmpLocation() {
        if (getAlliance() == Alliance.Blue) {
            return AbstractConstants.AMP_LOCATION_RED;
        } else {
            return AbstractConstants.AMP_LOCATION_BLUE;
        }
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------

    // #region: ----- Aimer -----
    public Tuple2<Rotation2d> getAimerAngleRange() {
        return new Tuple2<Rotation2d>(Rotation2d.fromDegrees(1), Rotation2d.fromDegrees(40));
    }

    public Rotation2d getAimerEncoderOffset() {
        return Rotation2d.fromRadians(2.599);
    }

    public int getAimerEncoderPort() {
        return uniqueRoboRioPort(5, RoboRioPortArrays.DIO);
    }

    public int getAimerMotorIdLeft() {
        return uniqueCanBusId(23, getCanivoreId());
    }

    public int getAimerMotorIdRight() {
        return uniqueCanBusId(22, getCanivoreId());
    }

    public PID getAimerPid() {
        return new PID(.6, 0, 0);
    }

    // #endregion

    // #region: ----- Canivore -----

    public static String getCanivoreId() {
        return "1559Canivore";
    }

    // #endregion

    // #region: ----- Climber --------
    public Measure<Distance> getClimberMaxHeight() {
        return Inches.of(12);
    }

    public int getClimberMotorIdLeft() {
        return uniqueCanBusId(25, getCanivoreId());
    }

    public int getClimberMotorIdRight() {
        return uniqueCanBusId(24, getCanivoreId());
    }

    public PID getClimberPid() {
        return new PID(.1, 0, 0);
    }

    // #endregion

    // #region: ----- Feeder -----
    public int getFeederMotorId() {
        return uniqueCanBusId(21, getCanivoreId());
    }

    public boolean isFeederMotorInverted() {
        return true;
    }

    public PID getFeederPidValues() {
        return new PID(0.33 / CONSTANTS.getFeederVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    public Measure<Velocity<Angle>> getFeederVelocityForward() {
        // TODO: Configure Value.
        return RevolutionsPerSecond.of(3666);
    }

    public Measure<Velocity<Angle>> getFeederVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
    }

    // #endregion

    // #region: ----- Flywheel -----
    public int getFlywheelMotorIdLeft() {
        return uniqueCanBusId(24, getCanivoreId());
    }

    public int getFlywheelMotorIdRight() {
        return uniqueCanBusId(25, getCanivoreId());
    }

    public double getFlywheelForwardVoltage() {
        // TODO: Configure Value.
        return 10;
    }

    public double getFlywheelReverseVoltage() {
        // TODO: Configure Value.
        return -6;
    }

    public double flywheelSpinOffset() {
        // TODO: Tune.
        return 1;
    }

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

    public boolean isIntakeMotorInverted() {
        return true;
    }

    public PID getIntakePidValues() {
        return new PID(0.33 / CONSTANTS.getIntakeVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    public Measure<Velocity<Angle>> getIntakeVelocityForward() {
        // TODO: Configure Value.
        return RevolutionsPerSecond.of(8250);
    }

    public Measure<Velocity<Angle>> getIntakeVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
    }

    // #endregion

    // #region: ----- LEDs -----

    public int getLedPort() {
        return uniqueRoboRioPort(0, RoboRioPortArrays.PWM);
    }

    public abstract int getLedLength();

    public Color[] getMotorOverheatEmergencyPattern() {
        return new Color[] { Color.kYellow, Color.kYellow, Color.kRed, Color.kRed, Color.kBlack, Color.kBlack };
    }

    // #endregion

    // #region: ----- Note Sensor -----

    public static int getNoteSensorChannel() {
        return uniqueRoboRioPort(2, RoboRioPortArrays.DIO);
    }

    // #endregion

    // #region: ----- roboRIO -----
    public abstract String getRoboRioSerialNumber();

    // #endregion

    // #region: ----- Swerve --------
    public abstract Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets();

    private static Map<WheelModuleIndex, SwerveModuleHardwareIds> swerveModuleHardwareIds = new HashMap<>(4) {
        {
            put(WheelModuleIndex.FRONT_LEFT,
                    new SwerveModuleHardwareIds(uniqueCanBusId(0, getCanivoreId()), uniqueCanBusId(1, getCanivoreId()),
                            uniqueCanBusId(2, getCanivoreId())));

            put(WheelModuleIndex.FRONT_RIGHT, new SwerveModuleHardwareIds(uniqueCanBusId(3, getCanivoreId()),
                    uniqueCanBusId(4, getCanivoreId()), uniqueCanBusId(5, getCanivoreId())));

            put(WheelModuleIndex.BACK_LEFT, new SwerveModuleHardwareIds(uniqueCanBusId(9, getCanivoreId()),
                    uniqueCanBusId(10, getCanivoreId()), uniqueCanBusId(11, getCanivoreId())));

            put(WheelModuleIndex.BACK_RIGHT, new SwerveModuleHardwareIds(uniqueCanBusId(6, getCanivoreId()),
                    uniqueCanBusId(7, getCanivoreId()), uniqueCanBusId(8, getCanivoreId())));
        }
    };

    public Map<WheelModuleIndex, SwerveModuleHardwareIds> getSwerveModuleHardwareIds() {
        return swerveModuleHardwareIds;
    }

    // #endregion

    // #region: ----- Traverser -----

    public int getTraverserMotorId() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Traverser");
    }

    public boolean isTraverserInverted() {
        return true;
    }

    public PID getTraverserPidValues() {
        return new PID(0.33 / CONSTANTS.getTraverserVelocity().in(RevolutionsPerSecond), 0, 0, 11.0 / 11000);
    }

    public Measure<Velocity<Angle>> getTraverserVelocity() {
        // TODO: Tune.
        return RevolutionsPerSecond.of(5000);
    }

    // #endregion

    // #region: ----- Vision -----

    public String getCameraNameBack() {
        return "limelight-back";
    }

    public String getCameraNameFront() {
        return "limelight";
    }

    // #endregion

    // #region: --------------- Motor / Motor Controller Settings --------------

    public double getMotorSafeTemperatureBuffer() {
        return 0.9;
    }

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
