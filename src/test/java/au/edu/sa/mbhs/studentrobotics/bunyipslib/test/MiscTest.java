package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Ref;

/**
 * Miscellaneous tests for BunyipsLib features.
 *
 * @author Lucas Bubner, 2025
 */
class MiscTest {
    @Test
    void testRefStringify() {
        assertEquals("item", Ref.stringify(Ref.of("item")));
        assertEquals("item2", Ref.stringify(Ref.of(Ref.of("item2"))));
        assertEquals("item3", Ref.stringify(Ref.of(Ref.of(Ref.of("item3")))));
    }
}
