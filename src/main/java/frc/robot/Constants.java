package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ========================= CONSTANTS ======================================
  // ---------- Operation Modes ----------
  public static final OperatingMode CURRENT_OPERATING_MODE = OperatingMode.REAL_WORLD;
  public static final NeutralModeValue WHEEL_BRAKE_MODE = NeutralModeValue.Brake;
  public static final boolean FIELD_RELATIVE = false;

  // ---------- Aliance ----------
  // This is the side of the field that the aumation path are made for.
  public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;
  public static final boolean FLIP_PATH_IF_ALLIANCE_IS_NOT_DEFAULT = true;

  // ---------- Driving Config ----------
  public static final double JOYSTICK_DEADBAND = 0.2;
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(3.0);

  // ---------- Robot Measurements ----------
  // Middle of front wheel to middle of back wheel.
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
  // Middle of left wheel to middle of right wheel.
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);
  public static final double WHEEL_DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double WHEEL_TURN_GEAR_RATIO = 12.8;

  // ---------- Ids ----------
  public static final String CANIVORE_BUS_ID = "1559Canivore";

  // ========================= Enums ==========================================
  public static enum OperatingMode {
    REAL_WORLD,
    SIMULATION,
    LOG_REPLAY
  }

  // ========================= Constructors ===================================
  /** Makes this class non-instantiable.*/
  private Constants() {}
}
