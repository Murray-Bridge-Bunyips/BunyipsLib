package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;

class UnaryFunctionTest {
    @Test
    void testApply() {
        UnaryFunction f = x -> x * x;
        assertEquals(1, f.apply(1));
        assertEquals(4, f.apply(2));
        assertEquals(2, f.apply(1.414), 1.0e-3);
        assertEquals(64, f.apply(-8));
    }

    @Test
    void testPipeTo() {
        UnaryFunction f = x -> x * x;
        UnaryFunction g = x -> x + 1;
        UnaryFunction h = f.pipeTo(g); // h(x) = g(f(x)) = x^2 + 1
        assertEquals(2, h.apply(1));
        assertEquals(65, h.apply(8));
    }

    @Test
    void testMult() {
        UnaryFunction f = x -> x * x;
        UnaryFunction g = x -> x + 1;
        UnaryFunction h = f.mult(g); // h(x) = f(x) * g(x) = (x^2)(x + 1) = x^3 + x^2
        assertEquals(2, h.apply(1));
        assertEquals(216 + 36, h.apply(6));
        assertEquals(12, h.apply(2));
    }

    @Test
    void testMultScalar() {
        UnaryFunction f = x -> x * x;
        double scalar = 2.5;
        UnaryFunction h = f.mult(scalar); // h(x) = 2.5 x^2
        assertEquals(250, h.apply(10));
        assertEquals(2.5, h.apply(1));
        assertEquals(2.5, h.apply(-1));
        assertEquals(10, h.apply(2));
    }

    @Test
    void testDivZeroDenominator() {
        UnaryFunction f = x -> x;
        UnaryFunction g = x -> 0;
        UnaryFunction h = f.div(g);
        assertEquals(0, h.apply(0));
        assertEquals(Double.POSITIVE_INFINITY, h.apply(1));
        assertEquals(Double.NEGATIVE_INFINITY, h.apply(-1));
    }

    @Test
    void testDivNonzeroDenominator() {
        UnaryFunction f = x -> 2 * x;
        UnaryFunction g = x -> x + 1;
        UnaryFunction h = f.div(g);
        assertEquals(0, h.apply(0));
        assertEquals(4, h.apply(-2));
        assertEquals(16.0 / 9, h.apply(8));
    }

    @Test
    void testDivScalar() {
        UnaryFunction f = x -> x * x;
        double scalar = 2.5;
        UnaryFunction h = f.div(scalar); // h(x) = x^2 / 2.5
        assertEquals(10, h.apply(5));
    }

    @Test
    void testExp() {
        UnaryFunction f = x -> x * x;
        UnaryFunction g = x -> x + 2;
        UnaryFunction h = f.exp(g); // h(x) = f(x) ^ g(x) = (x^2)^(x + 2)
        assertEquals(1, h.apply(1));
        assertEquals(0, h.apply(0));
        assertEquals(1, h.apply(-2));
        assertEquals(256, h.apply(2)); // (2^2)^(2 + 2) = 4^4
        assertEquals(59049, h.apply(3)); // (3^2)^(3 + 2) = 9^5
    }

    @Test
    void testExpScalar() {
        UnaryFunction f = x -> x * x;
        double scalar = 4;
        UnaryFunction h = f.exp(scalar); // h(x) = f(x)^4 = x^8
        assertEquals(0, h.apply(0));
        assertEquals(1, h.apply(1));
        assertEquals(1, h.apply(-1));
        assertEquals(256, h.apply(2));
        assertEquals(6561, h.apply(3));
    }
}