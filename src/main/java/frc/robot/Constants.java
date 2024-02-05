package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything
 * functional in this class.
 */
public final class Constants {

  // ========================= Enums ==========================================
  public static enum OperatingMode {
    REAL_WORLD,
    SIMULATION,
    LOG_REPLAY
  }

  // ========================= CONSTANTS ======================================
  // ---------- Operation Modes ----------
  public static final OperatingMode CURRENT_OPERATING_MODE = OperatingMode.REAL_WORLD;
  public static final NeutralModeValue WHEEL_BRAKE_MODE = NeutralModeValue.Brake;
  public static final boolean FIELD_RELATIVE = false;

  // ---------- Alliance ----------
  // This is the side of the field that the aumation path are made for.
  public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;
  public static final boolean FLIP_PATH_IF_ALLIANCE_IS_NOT_DEFAULT = true;

  // ---------- Driving Config ----------
  public static final double JOYSTICK_DEADBAND = 0.2;
  public static final double MAX_LINEAR_SPEED_IN_METERS_PER_SECOND = 3.0;
  public static final double ADVANTAGE_ODOMETRY_LOG_FREQUENCY = 100.0;
  public static final double ADVANTAGE_DEFAULT_LOG_FREQUENCY = 50.0;

  // ---------- Robot Measurements ----------
  // Middle of front wheel to middle of back wheel.
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
  // Middle of left wheel to middle of right wheel.
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);

  // ---------- Wheel & GHear Measurements ----------
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double WHEEL_DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 /
      27.0) * (45.0 / 15.0); // L2 Gear ratio
  // public static final double WHEEL_DRIVE_GEAR_RATIO = 6.12; // L3 Gear ratio
  public static final double WHEEL_TURN_GEAR_RATIO = 12.8;

  // ---------- Wheel Rotation Offsets ----------
  // Note: Chaning the Offset by Pie (180 degrees) will invert the direction the
  // wheel spins.
  public static final Rotation2d FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromRotations(-0.300293);
  public static final Rotation2d FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromRotations(-0.228760);
  public static final Rotation2d BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromRotations(-0.238525);
  public static final Rotation2d BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromRotations(-0.000732);

  // ---------- Game Piece Handling -------
  public static final double INTAKE_FORWARD_VOLTAGE = 6.0;
  public static final double INTAKE_REVERSE_VOLTAGE = -6.0;
  public static final double FEEDER_FORWARD_VOLTAGE = 6.0; // TODO: Configure Value.
  public static final double FEEDER_REVERSE_VOLTAGE = -FEEDER_FORWARD_VOLTAGE;
  public static final double FLYWHEEL_REVERSE_VOLTAGE = -6.0; // TODO: Configure Value.

  public static final double AIMER_KP = 0; // TODO
  public static final double AIMER_KI = 0;
  public static final double AIMER_KD = 0;
  public static final double AIMER_ANGLE_OFFSET = 0; // TODO/calibrate

  // ---------- Hardware Config --------
  public static final boolean HAVE_FEEDER = false;
  public static final boolean HAVE_INTAKE = false;
  public static final boolean HAVE_AIMER = false;
  public static final boolean HAVE_FLYWHEEL = false;
  public static final boolean HAVE_COLOR_SENSOR = false;
  public static final boolean HAVE_SHOOTER = HAVE_FEEDER && HAVE_INTAKE && HAVE_AIMER && HAVE_FLYWHEEL
      && HAVE_COLOR_SENSOR;

  // ---------- Hardware Ids ----------
  // --- Camera ---
  public static final String CAMERA_1_NAME = "front"; // TODO: Set Value.
  // --- Canivore ---
  public static final String CANIVORE_BUS_ID = "1559Canivore";
  // --- Flywheel ---
  public static final int INTAKE_L_ID = 20;
  public static final int INTAKE_R_ID = 21;
  public static final int FEEDER_L_ID = 22;
  public static final int FEEDER_R_ID = 23;
  public static final int AIMER_L_ID = 24;
  public static final int AIMER_R_ID = 25;
  public static final int FLYWHEEL_L_ID = 26;
  public static final int FLYWHEEL_R_ID = 27;

  // --- Gyro ---
  public static final int BASE_GYRO_ID = 12;
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

  // ---------- Digital IO Ports ----------
  public static final int AIMER_ENCODER_PORT = 0;

  // ---------- Adressable LEDs ----------
  public static final int ADDRESSABLE_LED_PORT = 0;
  public static final int ADDRESSABLE_LED_LENGTH = 144;

  // ---------- Color Sensor ----------
  public static final int COLOR_SENSOR_PROXIMITY_THRESHOLD = 300; // TODO: Configure Value.
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
