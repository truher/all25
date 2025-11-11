package org.team100.lib.subsystems.five_bar;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class FiveBarMechTest {
    @Test
    void testWide() {
        assertFalse(FiveBarMech.feasible(0, Math.PI));
    }

    @Test
    void testNarrow() {
        assertFalse(FiveBarMech.feasible(Math.PI, 0));
    }

    @Test
    void testOK() {
        assertTrue(FiveBarMech.feasible(Math.PI / 2, Math.PI / 2));
    }

}
