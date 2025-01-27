// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units;

/**
 * Unit of energy dimension.
 *
 * <p>This is the base type for units of energy dimension. It is also used to specify the dimension
 * for {@link Measure}: {@code Measure<Energy>}.
 *
 * <p>Actual units (such as {@link Units#Joules} and {@link Units#Kilojoules}) can be found in the
 * {@link Units} class.
 *
 * @since 1.0.0-pre
 */
public class Energy extends Unit<Energy> {
    Energy(Energy baseUnit, double baseUnitEquivalent, String name, String symbol) {
        super(baseUnit, baseUnitEquivalent, name, symbol);
    }

    Energy(Energy baseUnit, UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
    }
}
