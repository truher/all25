package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

public class DirectionSE2Test {
    @Test
    void testCourse() {
        // zero direction is an error
        assertThrows(IllegalArgumentException.class, () -> new DirectionSE2(0, 0, 0));
    }

}
