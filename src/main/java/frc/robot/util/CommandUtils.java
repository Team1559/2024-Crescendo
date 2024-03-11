package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandUtils {

    public static Command addName(Command command) {
        return addName(null, command);
    }

    public static Command addName(String prefix, Command command) {
        String name = StackWalker.getInstance()
                .walk(frames -> frames.skip(1).findFirst().map(StackWalker.StackFrame::getMethodName))
                .orElse("UNKNOWN");
        command.setName((prefix == null ? "" : prefix + ".") + name);
        return command;
    }
}
