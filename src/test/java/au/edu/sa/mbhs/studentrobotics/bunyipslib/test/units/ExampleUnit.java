package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Unit;

class ExampleUnit extends Unit<ExampleUnit> {
    ExampleUnit(double baseUnitEquivalent) {
        this(baseUnitEquivalent, "Example", "ex");
    }

    ExampleUnit(
            ExampleUnit baseUnit,
            UnaryFunction toBase,
            UnaryFunction fromBase,
            String name,
            String symbol) {
        super(baseUnit, toBase, fromBase, name, symbol);
    }

    ExampleUnit(double baseUnitEquivalent, String name, String symbol) {
        super(null, baseUnitEquivalent, name, symbol);
    }
}