package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OnboardAngularPositionServoTest implements Timeless {
    // note ridiculously precise delta
    private static final double kDelta = 1e-9;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final boolean kActuallyPrint = false;

    @Test
    void testOnboard() {
        final MockBareMotor turningMotor = new MockBareMotor(Feedforward100.makeSimple());
        final MockRotaryPositionSensor positionSensor = new MockRotaryPositionSensor();
        final RotaryMechanism mech = new RotaryMechanism(
                logger, turningMotor, positionSensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);
        servo.reset();
        // spin for 1 s
        for (int i = 0; i < 50; ++i) {
            servo.setPositionProfiled(1, 0);
            stepTime();
            if (kActuallyPrint)
                Util.printf("i: %d position: %5.3f %5.3f\n", i, turningMotor.position, turningMotor.velocity);
            // lets say we're on the profile.
            positionSensor.angle = servo.m_setpoint.x();
            positionSensor.rate = servo.m_setpoint.v();
        }
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.m_setpoint.x(), kDelta);
        assertEquals(1.0, servo.m_setpoint.v(), kDelta);
        assertEquals(0.5, positionSensor.getPositionRad().getAsDouble(), kDelta);
        assertEquals(1.000, turningMotor.velocity, kDelta);
    }

}
