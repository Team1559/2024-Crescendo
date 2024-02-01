package frc.robot;

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
  public static final double FEEDER_FORWARD_VOLTAGE = 6.0; // TODO: Configure Value.
  public static final double FEEDER_REVERSE_VOLTAGE = -FEEDER_FORWARD_VOLTAGE;
  public static final double FLYWHEEL_REVERSE_VOLTAGE = -6.0; // TODO: Configure Value.

  // ---------- Hardware Ids ----------
  // --- Camera ---
  public static final String CAMERA_1_NAME = "front"; // TODO: Set Value.
  // --- Canivore ---
  public static final String CANIVORE_BUS_ID = "1559Canivore";
  // --- Flywheel ---
  public static final int FLYWHEEL_L_ID = 0; // TODO: Set Value.
  public static final int FLYWHEEL_R_ID = 0; // TODO: Set Value.
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

  // ---------- Roborio ----------
  public static final int LEFT_FEED_MOTOR_ID = 20;
  public static final int RIGHT_FEED_MOTOR_ID = 21;

  // ---------- Adressable LEDs ----------
  public static final int ADDRESSABLE_LED_PORT = 0;
  public static final int ADDRESSABLE_LED_LENGTH = 144;

  // ---------- Color Sensor ----------
  public static final int COLOR_SENSOR_V3_NO_OBJECT_PROXIMITY = 300; // TODO: Configure Value.

  // ========================= Enums ==========================================
  public static enum OperatingMode {
    REAL_WORLD,
    SIMULATION,
    LOG_REPLAY
  }

  // ========================= Constructors ===================================
  /** Makes this class non-instantiable. */
  private Constants() {
  }
}
