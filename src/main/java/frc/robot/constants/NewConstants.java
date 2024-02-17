package frc.robot.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;

public abstract class NewConstants {

    // ========================= Enums =========================================
    public static enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    // ========================= Static CONSTANTS ==============================
    public static final boolean FORCE_GAME_ROBOT_CONSTANTS = false;
    private static final NewConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final NewConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    // ========================= Static Methods ================================
    public static boolean isGameRobot() {
        return FORCE_GAME_ROBOT_CONSTANTS
                || RobotController.getSerialNumber().equals(GAME_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || !RobotController.getSerialNumber().equals(TEST_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || DriverStation.getMatchType() != MatchType.None
                || DriverStation.getMatchNumber() != 0
                || DriverStation.getEventName() == null
                || Objects.equals(DriverStation.getEventName(), "");
    }

    public static NewConstants get() {
        if (isGameRobot()) {
            return GAME_ROBOT_CONSTANTS;
        } else {
            return TEST_ROBOT_CONSTANTS;
        }
    }

    // ========================= Methods =======================================
    // ---------- Operation Modes ----------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();

    // ---------- Hardware ----------
    public abstract String getRoboRioSerialNumber();
}
