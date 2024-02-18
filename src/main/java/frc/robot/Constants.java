package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything
 * functional in this class.
 */
@SuppressWarnings("unused") // < -- Never do this (Gets rid of dead code warnings)
public final class Constants {

    // ========================= CONSTANTS ======================================
    // ---------- Driving Config ----------
    public static final double JOYSTICK_DEADBAND = 0.2;
    public static final double MAX_LINEAR_SPEED_IN_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS = Constants.MAX_LINEAR_SPEED_IN_METERS_PER_SECOND
            / CONSTANTS.getWheelRadius().in(Meters);
    public static final double ADVANTAGE_ODOMETRY_LOG_FREQUENCY = 100.0;
    public static final double ADVANTAGE_DEFAULT_LOG_FREQUENCY = 50.0;

    // ---------- Wheel & GHear Measurements ----------
    public static final double WHEEL_DRIVE_GEAR_RATIO_L3 = (50.0 / 14.0) * (16.0 /
            28.0) * (45.0 / 15.0); // L3 Gear ratio
    public static final double WHEEL_DRIVE_GEAR_RATIO_L2 = (50.0 / 14.0) * (17.0 /
            27.0) * (45.0 / 15.0); // L2 Gear ratio
    // public static final double WHEEL_DRIVE_GEAR_RATIO = 6.12; // L3 Gear ratio
    public static final double WHEEL_TURN_GEAR_RATIO = 12.8;

    // ---------- Game Piece Handling -------
    public static final double INTAKE_FORWARD_VOLTAGE = 6.0;
    public static final double INTAKE_REVERSE_VOLTAGE = -6.0;
    public static final double FEEDER_FORWARD_VOLTAGE = 6.0; // TODO: Configure Value.
    public static final double FEEDER_REVERSE_VOLTAGE = -FEEDER_FORWARD_VOLTAGE;
    public static final double FLYWHEEL_FOWARDS_VOLTAGE = 9.0; // TODO: Configure Value
    public static final double FLYWHEEL_REVERSE_VOLTAGE = -6.0; // TODO: Configure Value.
    // TODO - Revert Flywheel Voltage to 12
    public static final double AIMER_KP = .4;
    public static final double AIMER_KI = 0;
    public static final double AIMER_KD = 0;
    public static final Rotation2d AIMER_ANGLE_OFFSET = Rotation2d.fromRadians(2.599); // TODO/calibrate
    public static final double AIMER_LOWER_ANGLE = 1;
    public static final double AIMER_UPPER_ANGLE = 45;

    // ---------- Hardware Config ----------
    // --- Aimer ---
    public static final int AIMER_LEFT_MOTOR_ID = 23;
    public static final int AIMER_RIGHT_MOTOR_ID = 22;
    // --- Camera ---
    public static final String SHOOTER_CAMERA_NAME = "limelight";
    // --- Canivore ---
    public static final String CANIVORE_BUS_ID = "1559Canivore";
    // --- Feeder ---
    public static final int FEEDER_MOTOR_ID = 21;
    public static final boolean IS_FEEDER_INVERTED = true;
    // --- Flywheel ----
    public static final int FLYWHEEL_LEFT_MOTOR_ID = 24;
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 25;
    // --- Gyro ---
    public static final int BASE_GYRO_ID = 12;
    // --- Intake ---
    public static final int INTAKE_MOTOR_ID = 20;
    public static final boolean IS_INTAKE_INVERTED = true;
    // --- Swerve Drives ---
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
    public static final int FRONT_RIGHT_CANCODER_ID = 5;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 9;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 10;
    public static final int BACK_LEFT_CANCODER_ID = 11;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    public static final int BACK_RIGHT_CANCODER_ID = 8;
    public static final NeutralModeValue SWERVE_WHEEL_BRAKE_MODE = NeutralModeValue.Brake;

    // ---------- Digital IO Ports ----------
    public static final int AIMER_ENCODER_PORT = 0;

    // ---------- Adressable LEDs ----------
    public static final int ADDRESSABLE_LED_PORT = 0;
    public static final int ADDRESSABLE_LED_LENGTH = 144;

    // ---------- Color Sensor ----------
    public static final int COLOR_SENSOR_PROXIMITY_THRESHOLD = 1500; // TODO: Configure Value.

    // ---------- Power Constants ----------
    public static final int NEO_SPARK_BRUSHLESS_CURRENT_LIMIT = 24;
    public static final int NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT = 80;

    // ========================= Configuration Objects ========================
    /**
     * Allow 40A continuous, 80A momentary supply current. See: <a href=
     * "https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html">CTR
     * Electronics: CurrentLimitsConfigs</a>
     * 
     * @return A {@link CurrentLimitsConfigs} object with the default Current
     *         limits.
     */
    public static CurrentLimitsConfigs getDefaultCurrentLimitsConfig() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40.0;
        limits.SupplyCurrentThreshold = 80.0;
        limits.SupplyTimeThreshold = 0.5;
        return limits;
    }

    // ========================= Constructors ===================================
    /** Makes this class non-instantiable. */
    private Constants() {
    }
}
