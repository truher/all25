package org.team100.lib.subsystems.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class CalibratedServoTest {

    @Test
    void testRoundTrip() {
        // the servo round trip is not precise.
        AffineFunction f = new AffineFunction(-3.216, 1.534);
        try (CalibratedServo s = new CalibratedServo(0,
                new Clamp(-Math.PI, Math.PI),
                f)) {
            double xx = f.x(0.0);
            assertEquals(0.47699004975124376, xx, 1e-12);
            s.set(0.0);
            assertEquals(0.0, s.get(), 1e-12);
        }
    }
}
