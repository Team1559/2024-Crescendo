package org.victorrobotics.frc.util;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class ReflectionUtils {

    /** Will attempt to change a static final field and log any errors. */
    public static void modifyStaticFinalField(Class<?> classContainingField, String fieldName, Object newValue) {

        Field field, modifiers;
        try {
            field = classContainingField.getDeclaredField(fieldName);

            modifiers = field.getClass().getDeclaredField("modifiers");
            modifiers.setAccessible(true);

            modifiers.setInt(field, field.getModifiers() & ~Modifier.FINAL);
            field.setAccessible(true);

            field.set(null, newValue);

            modifiers.setInt(field, field.getModifiers() & Modifier.FINAL);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
