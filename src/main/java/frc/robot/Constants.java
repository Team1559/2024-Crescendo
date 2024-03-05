package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.base.SwerveModule.WheelModuleIndex;

public class Constants {

    // #region: ================ Enums =========================================
    public enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    private enum RoboRioPortArrays {
        DIO,
        PWM
    }

    // #endregion

    // #region: ================ Static Classes ================================
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

    // #endregion

    // ========================= Static CONSTANTS ==============================

    // ========================= Static Variables ==============================
    private static Map<String, Set<Integer>> uniqueCanBusIds;
    private static Map<RoboRioPortArrays, Set<Integer>> uniqueRoboRioPorts;

    // #region: ================ Static Methods ================================
    @SuppressWarnings("unused")
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
            throw new RuntimeException("Duplicate roboRIO Port (" + port + ") on the " + portArray + " array!");
        }

        return port;
    }

    // #endregion

    // ==================== "CONSTANTS" (Ctrl + K, Ctrl + 8 to fold regions) ===
    // #region: --------------- Game Robot / Technician ------------------------
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = true;

    @SuppressWarnings("unused")
    private static boolean isGameRobot() {
        String roboRioSerialNumber = System.getenv("serialnum");
        roboRioSerialNumber = roboRioSerialNumber == null ? "" : roboRioSerialNumber.trim();

        return FORCE_GAME_ROBOT_CONSTANTS
                || roboRioSerialNumber.equalsIgnoreCase("03282B9F" /* Game Robot */)
                || !roboRioSerialNumber.equalsIgnoreCase("03282BB6" /* Test Robot */);
    }

    // #endregion

    // #region: --------------- Alliance ---------------------------------------
    /**
     * @return The assigned Alliance or the {@link #getDefaultAllianceForAuto}, if
     *         no alliance is set.
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(getDefaultAllianceForAuto());
    };

    /**
     * @return The side of the field that the automation paths are made for.
     */
    public static Alliance getDefaultAllianceForAuto() {
        return Alliance.Blue;
    }

    public static boolean shouldFlipPath() {
        return getAlliance() != getDefaultAllianceForAuto() && Constants.shouldFlipPathIfAssignedAllianceIsNotDefault();
    }

    public static boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
    public static boolean hasAimerSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasClimberSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasNoteSensorSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasFeederSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasFlywheelSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasIntakeSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasVisionSubsystem() {
        if (isGameRobot()) {
            return true;
        } else {
            return true;
        }
    }

    public static boolean hasTraverserSubsystem() {
        if (isGameRobot()) {
            return false;
        } else {
            return false;
        }
    }

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public static double getJoystickDeadband() {
        return 0.05;
    }

    public static Measure<Velocity<Angle>> getMaxAngularSpeed() {
        if (isGameRobot()) {
            // TODO: Tune.
            return DegreesPerSecond.of(360);
        } else {
            return RadiansPerSecond.of(360);
        }
    }

    public static Measure<Velocity<Distance>> getMaxLinearSpeed() {
        if (isGameRobot()) {
            // TODO: Tune.
            return MetersPerSecond.of(5);
        } else {
            // TODO: Tune.
            return MetersPerSecond.of(10);
        }
    }

    // #endregion

    // #region: --------------- Game Objects -----------------------------------
    private static final Translation3d SPEAKER_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));
    private static final Translation3d SPEAKER_LOCATION_RED = new Translation3d(Units.inchesToMeters(652.73),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80.5));

    public static Translation3d getSpeakerLocation() {
        if (getAlliance() == Alliance.Blue) {
            return Constants.SPEAKER_LOCATION_BLUE;
        } else {
            return Constants.SPEAKER_LOCATION_RED;
        }
    }

    private static final Translation3d AMP_LOCATION_RED = new Translation3d(Units.inchesToMeters(72.5),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));
    private static final Translation3d AMP_LOCATION_BLUE = new Translation3d(Units.inchesToMeters(578.77),
            Units.inchesToMeters(323.00),
            Units.inchesToMeters(44));

    public static Translation3d getAmpLocation() {
        if (getAlliance() == Alliance.Blue) {
            return Constants.AMP_LOCATION_RED;
        } else {
            return Constants.AMP_LOCATION_BLUE;
        }
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------

    // #region: ----- Aimer -----
    public static Tuple2<Rotation2d> getAimerAngleRange() {
        return new Tuple2<Rotation2d>(Rotation2d.fromDegrees(1), Rotation2d.fromDegrees(40));
    }

    public static Rotation2d getAimerEncoderOffset() {
        return Rotation2d.fromRadians(2.599);
    }

    public static int getAimerEncoderPort() {
        return uniqueRoboRioPort(5, RoboRioPortArrays.DIO);
    }

    public static Rotation2d getAimerErrorThreshold() {
        return Rotation2d.fromDegrees(2);
    }

    public static int getAimerMotorIdLeft() {
        return uniqueCanBusId(23, getCanivoreId());
    }

    public static int getAimerMotorIdRight() {
        return uniqueCanBusId(22, getCanivoreId());
    }

    public static PID getAimerPid() {
        return new PID(.6, 0, 0);
    }

    // #endregion

    // #region: ----- Camera -----

    public static String getCameraNameBack() {
        return "limelight-back";
    }

    public static String getCameraNameFront() {
        return "limelight";
    }

    public static Measure<Distance> getCameraLinearStandardDeviation() {
        return Meters.of(0.5);
    }

    public static Measure<Angle> getCameraRotationalStandardDeviation() {
        return Degrees.of(1);
    }

    // #endregion

    // #region: ----- Canivore -----

    public static String getCanivoreId() {
        return "1559Canivore";
    }

    // #endregion

    // #region: ----- Climber --------
    public static Measure<Distance> getClimberMaxHeight() {
        return Inches.of(12);
    }

    public static int getClimberMotorIdLeft() {
        return uniqueCanBusId(25, getCanivoreId());
    }

    public static int getClimberMotorIdRight() {
        return uniqueCanBusId(24, getCanivoreId());
    }

    public static PID getClimberPid() {
        return new PID(.1, 0, 0);
    }

    // #endregion

    // #region: ----- Feeder -----
    public static int getFeederMotorId() {
        return uniqueCanBusId(21, getCanivoreId());
    }

    public static boolean isFeederMotorInverted() {
        return true;
    }

    public static PID getFeederPidValues() {
        return new PID(0.33 / Constants.getFeederVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    public static Measure<Velocity<Angle>> getFeederVelocityForward() {
        // TODO: Configure Value.
        return getIntakeVelocityForward().divide(4);
    }

    public static Measure<Velocity<Angle>> getFeederVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
    }

    // #endregion

    // #region: ----- Flywheel -----
    public static int getFlywheelMotorIdLeft() {
        return uniqueCanBusId(24, getCanivoreId());
    }

    public static int getFlywheelMotorIdRight() {
        return uniqueCanBusId(25, getCanivoreId());
    }

    public static Measure<Voltage> getFlywheelForwardVoltage() {
        // TODO: Configure Value.
        return Volts.of(10);
    }

    public static Measure<Voltage> getFlywheelReverseVoltage() {
        // TODO: Configure Value.
        return Volts.of(-6);
    }

    public static double flywheelSpinOffset() {
        // TODO: Tune.
        return 1;
    }

    // #endregion

    // #region: ----- Gyro -----

    public static int getGyroId() {
        return uniqueCanBusId(12, getCanivoreId());
    }

    // #endregion

    // #region: ----- Intake -----

    public static int getIntakeMotorId() {
        return uniqueCanBusId(20, getCanivoreId());
    }

    public static boolean isIntakeMotorInverted() {
        return true;
    }

    public static PID getIntakePidValues() {
        return new PID(0.33 / Constants.getIntakeVelocityForward().in(RevolutionsPerSecond), 0, 0, 1.0 / 11000);
    }

    public static Measure<Velocity<Angle>> getIntakeVelocityForward() {
        // TODO: Configure Value.
        return RevolutionsPerSecond.of(11000);
    }

    public static Measure<Velocity<Angle>> getIntakeVelocityReverse() {
        // TODO: Configure Value.
        return getFeederVelocityForward().negate();
    }

    // #endregion

    // #region: ----- LEDs -----

    public static int getLedPort() {
        return uniqueRoboRioPort(0, RoboRioPortArrays.PWM);
    }

    public static int getLedLength() {
        if (isGameRobot()) {
            return 144 /* 1 strip */ * 3;
        } else {
            return 144 /* 1 strip */ * 1;
        }
    }

    public static Color[] getMotorOverheatEmergencyPattern() {
        return new Color[] { Color.kYellow, Color.kYellow, Color.kRed, Color.kRed, Color.kBlack, Color.kBlack };
    }

    // #endregion

    // #region: ----- Note Sensor -----

    public static int getNoteSensorChannel() {
        return uniqueRoboRioPort(2, RoboRioPortArrays.DIO);
    }

    // #endregion

    // #region: ----- Swerve --------
    public static Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets() {
        if (isGameRobot()) {
            return new HashMap<>() {
                {
                    put(WheelModuleIndex.FRONT_LEFT, Rotation2d.fromRadians(0.120));
                    put(WheelModuleIndex.FRONT_RIGHT, Rotation2d.fromRadians(-0.023));
                    put(WheelModuleIndex.BACK_LEFT, Rotation2d.fromRadians(2.789));
                    put(WheelModuleIndex.BACK_RIGHT, Rotation2d.fromRadians(0.853));
                }
            };
        } else {
            return new HashMap<>() {
                {
                    put(WheelModuleIndex.FRONT_LEFT, Rotation2d.fromRotations(-0.300293));
                    put(WheelModuleIndex.FRONT_RIGHT, Rotation2d.fromRotations(-0.228760));
                    put(WheelModuleIndex.BACK_LEFT, Rotation2d.fromRotations(-0.238525));
                    put(WheelModuleIndex.BACK_RIGHT, Rotation2d.fromRotations(-0.000732));
                }
            };
        }
    }

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

    public static Map<WheelModuleIndex, SwerveModuleHardwareIds> getSwerveModuleHardwareIds() {
        return swerveModuleHardwareIds;
    }

    // #endregion

    // #region: ----- Traverser -----

    public static int getTraverserMotorId() {
        // TODO: Add ID
        throw new UnsupportedOperationException("No Motor ID for Traverser");
    }

    public static boolean isTraverserInverted() {
        return true;
    }

    public static PID getTraverserPidValues() {
        return new PID(0.33 / Constants.getTraverserVelocity().in(RevolutionsPerSecond), 0, 0, 11.0 / 11000);
    }

    public static Measure<Velocity<Angle>> getTraverserVelocity() {
        // TODO: Tune.
        return RevolutionsPerSecond.of(5000);
    }

    // #endregion

    // #region: --------------- Motor / Motor Controller Settings --------------

    public static double getMotorSafeTemperatureBuffer() {
        return 0.9;
    }

    // #region: ----- NEO 550 Brushless Motor -----

    public static Measure<Current> getNeo550BrushlessCurrentLimit() {
        return Amps.of(24);
    }

    public static Measure<Current> getNeo550BrushlessCurrentSecondaryLimit() {
        return Amps.of(24);
    }

    // #endregion

    // #region: ----- Falcon 500 Motor -----

    @SuppressWarnings("unchecked")
    public static Map<String, StatusSignal<Boolean>> getAllGetFaultStatusSignalMethods(TalonFX motor) {
        Map<String, StatusSignal<Boolean>> faults = new HashMap<>();
        Class<?> c = motor.getClass();
        Method[] publicMethods = c.getMethods();
        for (int i = 0; i < publicMethods.length; i++) {
            Method method = publicMethods[i];
            String[] parts = method.toString().split("_");
            if (parts.length == 2 && parts[0].equals("public static StatusSignal<Boolean> getFault")) {
                try {
                    faults.put(parts[1].split("(")[0], (StatusSignal<Boolean>) method.invoke(motor));
                } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                    // Ignore.
                    e.printStackTrace();
                }
            }
        }
        return faults;
    }

    /**
     * See: <a href=
     * "https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf">Falcon
     * 500 temperature test under maximum load conditions</a>
     * 
     * @return 109 degrees C
     */
    public static Measure<Temperature> getFalcon500MaxTemperature() {
        return Celsius.of(109);
    }

    public static String[] getFaults(Map<String, StatusSignal<Boolean>> faultStatusSignals) {

        List<String> faults = new LinkedList<>();
        for (Entry<String, StatusSignal<Boolean>> entry : faultStatusSignals.entrySet()) {
            if (entry.getValue().getValue()) {
                faults.add(entry.getKey());
            }
        }
        return faults.toArray(new String[0]);
    }

    // #endregion

    // #region: ----- TalonFX Motor -----

    public static TalonFXConfiguration getDefaultTalonFXConfiguration(InvertedValue invertedValue,
            NeutralModeValue breakingMode) {

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 40.0;
        currentLimitsConfigs.SupplyCurrentThreshold = 80.0;
        currentLimitsConfigs.SupplyTimeThreshold = 0.5;
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

        MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
        driveMotorOutputConfigs.Inverted = invertedValue;
        driveMotorOutputConfigs.NeutralMode = breakingMode;
        talonFXConfiguration.withMotorOutput(driveMotorOutputConfigs);

        return talonFXConfiguration;
    }

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    public static OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public static boolean isDrivingModeFieldRelative() {
        if (isGameRobot()) {
            return true;
        } else {
            return false;
        }
    }

    // #endregion

    // #region: --------------- PathPlanner Settings ---------------------------
    /**
     * @return Value in Times per Second.
     */
    public static double getPathPlannerLogUpdateFrequencyDefault() {
        return 50;
    }

    /**
     * This higher rate is needed by PathPlanner.
     * 
     * @return Value in Times per Second.
     */
    public static double getPathPlannerLogFrequencyForOdometry() {
        return 100;
    }

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    public static Measure<Distance> getDriveBaseWheelRadius() {
        return Meters.of(Math.hypot(getWheelDistanceFrontToBack().divide(2).in(Meters),
                getWheelDistanceLeftToRight().divide(2).in(Meters)));
    }

    public static double getGearRatioOfDriveWheel() {
        if (isGameRobot()) {
            // L3 Gear ratio.
            return (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        } else {
            // L2 Gear ratio.
            return (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        }
    }

    public static double getGearRatioOfTurnWheel() {
        return 12.8;
    }

    /**
     * @return The distance between the middle of the front wheel to middle of the
     *         back wheel (X coordinates).
     */
    public static Measure<Distance> getWheelDistanceFrontToBack() {
        if (isGameRobot()) {
            return Inches.of(23);
        } else {
            return Inches.of(24);
        }
    }

    /**
     * @return The distance between the middle of the left wheel to middle of the
     *         right wheel (y coordinates).
     */
    public static Measure<Distance> getWheelDistanceLeftToRight() {
        if (isGameRobot()) {
            return Inches.of(23);
        } else {
            return Inches.of(24);
        }
    }

    public static Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }

    // #endregion
}
