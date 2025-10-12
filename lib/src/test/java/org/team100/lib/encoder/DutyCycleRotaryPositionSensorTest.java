package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.simulation.DutyCycleSim;

/** Uses DutyCycleSim. */
public class DutyCycleRotaryPositionSensorTest  {
    private static final double DELTA = 0.001;

    @Test
    void testSimple() {
        // make sure this thing works at all
        LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

        // this sleeps a long time so step ahead after construction
        AS5048RotaryPositionSensor s = new AS5048RotaryPositionSensor(
                log, new RoboRioChannel(0), 0, EncoderDrive.DIRECT);

        DutyCycleSim d = new DutyCycleSim(s.getDutyCycle());
        d.setFrequency(1000);
        d.setInitialized(true);
     

        d.setOutput(0.5);
     
        Cache.refresh();
        assertEquals(0.5, s.getRatio(), DELTA);
        assertEquals(3.135, s.getWrappedPositionRad(), DELTA);

        d.setOutput(0.8);
        // nothing happens until the cache refreshes
        assertEquals(0.5, s.getRatio(), DELTA);
        assertEquals(3.135, s.getWrappedPositionRad(), DELTA);
      
        Cache.refresh();
        assertEquals(0.8, s.getRatio(), DELTA);
        assertEquals(-1.252, s.getWrappedPositionRad(), DELTA);

        assertEquals(0, s.getTurns());
    }

    @Test
    void testWrapping() {
        LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

        AS5048RotaryPositionSensor s = new AS5048RotaryPositionSensor(
                log, new RoboRioChannel(0), 0, EncoderDrive.DIRECT);

        DutyCycleSim d = new DutyCycleSim(s.getDutyCycle());
        d.setFrequency(1000);
        d.setInitialized(true);
        d.setOutput(0.5);
        Cache.refresh();

        assertEquals(0.5, s.getRatio(), DELTA);
        assertEquals(3.135, s.getWrappedPositionRad(), DELTA);
    }
}
