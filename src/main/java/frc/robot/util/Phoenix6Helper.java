package frc.robot.util;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

public class Phoenix6Helper {

    @SuppressWarnings("unchecked")
    public static Map<String, StatusSignal<Boolean>> getAllGetFaultStatusSignalMethods(ParentDevice motor) {
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

    public static String[] getFaults(Map<String, StatusSignal<Boolean>> faultStatusSignals) {

        List<String> faults = new LinkedList<>();
        for (Entry<String, StatusSignal<Boolean>> entry : faultStatusSignals.entrySet()) {
            if (entry.getValue().getValue()) {
                faults.add(entry.getKey());
            }
        }
        return faults.toArray(new String[0]);
    }
}
