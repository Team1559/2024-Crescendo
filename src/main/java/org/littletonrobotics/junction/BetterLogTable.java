package org.littletonrobotics.junction;

import edu.wpi.first.units.ImmutableMeasure;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;

public class BetterLogTable extends LogTable {

    public BetterLogTable(long timestamp) {
        super(timestamp);
    }

    @Override
    public <U extends Unit<U>> void put(String key, Measure<U> value) {
        String unitKey = getUnitKey(key, value.unit());
        this.put(unitKey, new LogValue(value.magnitude(), (String) null));
    }

    @Override
    public <U extends Unit<U>> Measure<U> get(String key, Measure<U> defaultValue) {
        String unitKey = getUnitKey(key, defaultValue.unit());
        double value = this.get(unitKey).getDouble(defaultValue.magnitude());
        return ImmutableMeasure.ofRelativeUnits(value, defaultValue.unit());
    }

    @Override
    public <U extends Unit<U>> MutableMeasure<U> get(String key, MutableMeasure<U> defaultValue) {
        String unitKey = getUnitKey(key, defaultValue.unit());
        double value = this.get(unitKey).getDouble(defaultValue.magnitude());
        return MutableMeasure.ofRelativeUnits(value, defaultValue.unit());
    }

    private <U extends Unit<U>> String getUnitKey(String key, U unit) {
        return key + ":" + unit.symbol();
    }
}
