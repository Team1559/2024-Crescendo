package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotController;

public abstract class NewConstants {

    // ========================= Enums =========================================
    public static enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    // ========================= Static CONSTANTS ==============================
    public static final boolean IS_RUNNING_TEST_ROBOT = false;
    private static final NewConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final NewConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    // ========================= Static Methods ================================
    public static NewConstants get() {
        System.out.println("roboRIO Serial Number: " + RobotController.getSerialNumber());
        return IS_RUNNING_TEST_ROBOT ? TEST_ROBOT_CONSTANTS : GAME_ROBOT_CONSTANTS;
    }

    // ========================= Methods =======================================
    // ---------- Operation Modes ----------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();
}
