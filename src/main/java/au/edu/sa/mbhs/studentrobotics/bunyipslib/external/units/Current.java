// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Watts;

import androidx.annotation.NonNull;

/**
 * Unit of electric current dimension.
 *
 * <p>This is the base type for units of current dimension. It is also used to specify the dimension
 * for {@link Measure}: {@code Measure<Current>}.
 *
 * <p>Actual units (such as {@link Units#Amps} and {@link Units#Milliamps}) can be found in the
 * {@link Units} class.
 *
 * @since 1.0.0-pre
 */
public class Current extends Unit<Current> {
    Current(Current baseUnit, double baseUnitEquivalent, String name, String symbol) {
        super(baseUnit, baseUnitEquivalent, name, symbol);
    }

    Current(Current baseUnit, UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
    }

    /**
     * Constructs a unit of power equivalent to this unit of electrical current multiplied by another
     * unit of voltage. For example, {@code Amps.times(Volts)} will return a unit of power equivalent
     * to one Watt; {@code Amps.times(Millivolts)} will return a unit of power equivalent to a
     * milliwatt, and so on.
     *
     * @param voltage the voltage unit to multiply by
     * @param name    the name of the resulting unit of power
     * @param symbol  the symbol used to represent the unit of power
     * @return the power unit
     */
    @NonNull
    public Power times(@NonNull Unit<Voltage> voltage, @NonNull String name, @NonNull String symbol) {
        return new Power(Watts, toBaseUnits(1) * voltage.toBaseUnits(1), name, symbol);
    }
}
