// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Watts;

import androidx.annotation.NonNull;

/**
 * Unit of electric voltage dimension.
 *
 * <p>This is the base type for units of voltage dimension. It is also used to specify the dimension
 * for {@link Measure}: {@code Measure<Voltage>}.
 *
 * <p>Actual units (such as {@link Units#Volts} and {@link Units#Millivolts}) can be found in the
 * {@link Units} class.
 *
 * @since 1.0.0-pre
 */
public class Voltage extends Unit<Voltage> {
    /**
     * @noinspection SameParameterValue
     */
    Voltage(Voltage baseUnit, double baseUnitEquivalent, String name, String symbol) {
        super(baseUnit, baseUnitEquivalent, name, symbol);
    }

    Voltage(
            Voltage baseUnit,
            UnaryFunction toBaseConverter,
            UnaryFunction fromBaseConverter,
            String name,
            String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
    }

    /**
     * Constructs a unit of power equivalent to this unit of voltage multiplied by another unit of
     * electrical current. For example, {@code Volts.times(Amps)} will return a unit of power
     * equivalent to one Watt; {@code Volts.times(Milliams)} will return a unit of power equivalent to
     * a milliwatt, and so on.
     *
     * @param current the current unit to multiply by
     * @param name    the name of the resulting unit of power
     * @param symbol  the symbol used to represent the unit of power
     * @return the power unit
     */
    @NonNull
    public Power times(@NonNull Unit<Current> current, @NonNull String name, @NonNull String symbol) {
        return new Power(Watts, toBaseUnits(1) * current.toBaseUnits(1), name, symbol);
    }
}
