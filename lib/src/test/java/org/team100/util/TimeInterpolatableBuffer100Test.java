package org.team100.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

class TimeInterpolatableBuffer100Test {
    private static final double DELTA = 0.001;

    static class Item implements Interpolatable<Item> {
        public final double value;

        public Item(double v) {
            value = v;
        }

        @Override
        public Item interpolate(Item endValue, double t) {
            return new Item(MathUtil.interpolate(value, endValue.value, t));
        }
    }

    /** It interpolates proportionally. */
    @Test
    void testSimple2() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(10);
        b.reset(0, new Item(0));
        assertEquals(0, b.get(0).get().value, DELTA);
        b.put(1, new Item(10));
        assertEquals(5, b.get(0.5).get().value, DELTA);
        assertEquals(7.5, b.get(0.75).get().value, DELTA);
    }

    /** For off-the-end requests, it returns the last item. */
    @Test
    void testOffTheEnd2() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(10);
        b.reset(0, new Item(0));
        assertEquals(0, b.get(1).get().value, DELTA);
        b.put(1, new Item(10));
        assertEquals(10, b.get(1.5).get().value, DELTA);
    }

    @Test
    void testNoInitialValue() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(10);
        assertTrue(b.get(0).isEmpty());
        assertTrue(b.tailMap(0, false).isEmpty());
        assertFalse(b.tooOld(0));
        assertNull(b.lowerEntry(0));
        assertNull(b.ceilingEntry(0));
        assertEquals(0, b.size());
        assertEquals(0, b.lastKey(), DELTA);
    }
}
