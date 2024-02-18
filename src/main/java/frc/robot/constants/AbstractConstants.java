package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
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

    // ========================= Methods =======================================
    // ---------- Operation Modes ----------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();

    // -------------------- Alliance --------------------
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

    // -------------------- Capabilities Flags --------------------
    public abstract boolean hasAimerSubsystem();

    public abstract boolean hasClimberSubsystem();

    public abstract boolean hasColorSensorSubsystem();

    public abstract boolean hasFeederSubsystem();

    public abstract boolean hasFlywheelSubsystem();

    public abstract boolean hasIntakeSubsystem();

    public abstract boolean hasShooterSubsystemGroup();

    public abstract boolean hasTraverserSubsystem();

    public abstract boolean hasVisionSubsystem();

    // -------------------- Game Objects --------------------
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

    // -------------------- Hardware --------------------
    // ----- roboRIO -----
    public abstract String getRoboRioSerialNumber();

    // ----- Swerve --------
    /**
     * The index of the Rotation matches the Index of the Module in Advaltage Scope.
     * <p>
     * Note: Offsetting by 180 degrees will invert the direction the wheel spins.
     * </p>
     */
    public abstract Rotation2d[] getSwerveModuleEncoderOffsets();

    // ----- Traverser -----
    public abstract double getTraverserFowardVoltage();

    public abstract double getTraverserReverseVoltage();

    public abstract int getTraverserMotorId();

    public abstract boolean isTraverserInverted();

    // -------------------- Physical Measurements --------------------
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

    public Measure<Distance> getWheelRadius() {
        return Meters.of(Math.hypot(getWheelDistanceFrontToBack().divide(2).in(Meters),
                getWheelDistanceLeftToRight().divide(2).in(Meters)));
    }
}
