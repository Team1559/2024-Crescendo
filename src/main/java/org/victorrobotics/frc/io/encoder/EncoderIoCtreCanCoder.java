package org.victorrobotics.frc.io.encoder;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.util.Phoenix6Helper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class EncoderIoCtreCanCoder implements EncoderIo {

    public final boolean isInverted;
    public final CANcoder canCoder;
    public Rotation2d offset;

    private final StatusSignal<Double> absolutePosition, relativePosition;
    private final Map<String, StatusSignal<Boolean>> faults;
    private final List<StatusSignal<?>> statusSignals = new LinkedList<>();
    private final StatusSignal<?>[] statusSignalArray;

    public EncoderIoCtreCanCoder(int id, String canbus, boolean isInverted, Rotation2d offset) {

        // ---------- Create & Configure Encoder ----------
        canCoder = new CANcoder(id, canbus);
        canCoder.getConfigurator().apply(new CANcoderConfiguration());
        this.isInverted = isInverted;
        setOffset(offset);

        // ---------- Define Loggable Fields ----------
        faults = Phoenix6Helper.getAllGetFaultStatusSignalMethods(canCoder);
        statusSignals.addAll(faults.values());

        statusSignals.add(absolutePosition = canCoder.getAbsolutePosition());
        statusSignals.add(relativePosition = canCoder.getPositionSinceBoot());

        // ---------- Optimize Bus Utilization ----------
        statusSignalArray = statusSignals.toArray(new StatusSignal[0]);
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.getPathPlannerLogUpdateFrequencyDefault(),
                statusSignalArray);
        canCoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EncoderIoInputs inputs) {

        BaseStatusSignal.refreshAll(statusSignalArray);

        inputs.faults = Phoenix6Helper.getFaults(faults);

        inputs.positionAbsolute = getAbsolutePosition();
        inputs.positionRelative = getRelativePosition();
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getAbsolutePosition() {

        absolutePosition.refresh();
        Rotation2d position = Rotation2d.fromRotations(absolutePosition.getValueAsDouble());

        if (isInverted) {
            position = position.unaryMinus();
        }

        return position.plus(offset);
    }

    @Override
    public Rotation2d getRelativePosition() {

        relativePosition.refresh();
        Rotation2d position = Rotation2d.fromRotations(relativePosition.getValueAsDouble());

        // Forces Rotation2d to determine if it has passed threshold (360 degrees).
        position.plus(Rotation2d.fromDegrees(0));

        return position;
    }

    @Override
    public boolean isInverted() {
        return isInverted;
    }

    @Override
    public void setOffset(Rotation2d offset) {
        this.offset = offset == null ? Rotation2d.fromRadians(0) : offset;
    }
}
