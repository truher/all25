package org.team100.lib.util.wave;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ParabolicWaveTest {
        private static final double DELTA = 0.001;

    @Test
    void testUnitAmplitudeAndPeriod() {
        double amplitude = 1;
        double period = 1;
        ParabolicWave w = new ParabolicWave(amplitude, period);
        assertEquals(0.000, w.applyAsDouble(0.000), DELTA);
        assertEquals(0.125, w.applyAsDouble(0.125), DELTA);
        assertEquals(0.500, w.applyAsDouble(0.250), DELTA);
        assertEquals(0.875, w.applyAsDouble(0.375), DELTA);
        assertEquals(1.000, w.applyAsDouble(0.500), DELTA);
        assertEquals(0.875, w.applyAsDouble(0.625), DELTA);
        assertEquals(0.500, w.applyAsDouble(0.750), DELTA);
        assertEquals(0.125, w.applyAsDouble(0.875), DELTA);
        assertEquals(0.000, w.applyAsDouble(1.000), DELTA);
    }

    @Test
    void testAmplitude() {
        double amplitude = 2;
        double period = 1;
        ParabolicWave w = new ParabolicWave(amplitude, period);
        assertEquals(0.000, w.applyAsDouble(0.000), DELTA);
        assertEquals(0.250, w.applyAsDouble(0.125), DELTA);
        assertEquals(1.000, w.applyAsDouble(0.250), DELTA);
        assertEquals(1.750, w.applyAsDouble(0.375), DELTA);
        assertEquals(2.000, w.applyAsDouble(0.500), DELTA);
        assertEquals(1.750, w.applyAsDouble(0.625), DELTA);
        assertEquals(1.000, w.applyAsDouble(0.750), DELTA);
        assertEquals(0.250, w.applyAsDouble(0.875), DELTA);
        assertEquals(0.000, w.applyAsDouble(1.000), DELTA);
    }

    @Test
    void testPeriod() {
        double amplitude = 1;
        double period = 2;
        ParabolicWave w = new ParabolicWave(amplitude, period);
        assertEquals(0.000, w.applyAsDouble(0.000), DELTA);
        assertEquals(0.125, w.applyAsDouble(0.250), DELTA);
        assertEquals(0.500, w.applyAsDouble(0.500), DELTA);
        assertEquals(0.875, w.applyAsDouble(0.750), DELTA);
        assertEquals(1.000, w.applyAsDouble(1.000), DELTA);
        assertEquals(0.875, w.applyAsDouble(1.250), DELTA);
        assertEquals(0.500, w.applyAsDouble(1.500), DELTA);
        assertEquals(0.125, w.applyAsDouble(1.750), DELTA);
        assertEquals(0.000, w.applyAsDouble(2.000), DELTA);
    }
    
}
