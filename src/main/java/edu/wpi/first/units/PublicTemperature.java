package edu.wpi.first.units;

public class PublicTemperature extends Temperature {
    public PublicTemperature(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name,
            String symbol) {
        super(toBaseConverter, fromBaseConverter, name, symbol);
    }
}
