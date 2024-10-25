package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.InterpolatedLookupTable;

class InterpolatedLookupTableTest {
    private InterpolatedLookupTable lut;

    @Test
    void testAddDuplicate() {
        lut = new InterpolatedLookupTable();
        lut.add(1, 1);
        lut.add(1, 3);
        lut.add(4, 1);
        try {
            lut.createLUT();
            //computing the spline
        } catch (IllegalArgumentException ex) {
            assertEquals("The control points must all have strictly increasing X values.", ex.getMessage());
        }
    }

    @Test
    void testCreateLUT() {
        lut = new InterpolatedLookupTable();
        lut.add(1, 1);
        lut.add(2, 2);
        lut.add(3, 3);
        lut.add(4, 4);
        lut.createLUT();
    }

    @Test
    void testGet() {
        lut = new InterpolatedLookupTable();
        lut.add(1, 1);
        lut.add(2, 2);
        lut.add(3, 3);
        lut.add(4, 4);
        lut.createLUT();
        assertEquals(2, lut.get(2), 0.000001);
    }

    @Test
    void testInvalidCreation() {
        lut = new InterpolatedLookupTable();
        try {
            lut.createLUT();
        } catch (IllegalArgumentException ex) {
            assertEquals("There must be at least two control points and the arrays must be of equal length.",
                    ex.getMessage());
        }
    }

    @Test
    void testLargeCase() {
        lut = new InterpolatedLookupTable();
        for (int i = -100; i <= 100; i++) {
            lut.add(i, i + 1);
        }
        lut.createLUT();
        assertEquals(lut.get(85.5), 86.5);
    }
}