package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OnboardAngularPositionServoTest implements Timeless {
    // note ridiculously precise delta
    private static final double DELTA = 1e-9;
    private static final boolean DEBUG = true;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testOnboard() {
        final MockBareMotor turningMotor = new MockBareMotor(Feedforward100.makeSimple());
        final MockRotaryPositionSensor positionSensor = new MockRotaryPositionSensor();
        final RotaryMechanism mech = new RotaryMechanism(
                logger, turningMotor, positionSensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);
        servo.reset();
        // spin for 1 s
        for (int i = 0; i < 50; ++i) {
            servo.setPositionProfiled(1, 0);
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n", i, turningMotor.position, turningMotor.velocity);
            // lets say we're on the profile.
            positionSensor.angle = servo.m_nextUnwrappedSetpoint.x();
            positionSensor.rate = servo.m_nextUnwrappedSetpoint.v();
        }
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.m_nextUnwrappedSetpoint.x(), DELTA);
        assertEquals(1.0, servo.m_nextUnwrappedSetpoint.v(), DELTA);
        assertEquals(0.5, positionSensor.getWrappedPositionRad(), DELTA);
        assertEquals(1.000, turningMotor.velocity, DELTA);
    }

    /** This takes the short path. */
    @Test
    void testShortWayOnboardProfiled() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        IncrementalProfile profile = new TrapezoidIncrementalProfile(2, 2, 0.05);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);

        // at zero
        servo.reset();
        servo.periodic();
        stepTime();

        // move to the starting point of -3
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(-3, -150)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-1.5, -150), new Control100(-3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // start motionless at -3
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-3, servo.getWrappedPositionRad(), DELTA);

        // move to 3
        for (int i = 0; i < 17; ++i) {
            servo.setPositionProfiled(3, 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f velocity: %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // heading towards -pi
        assertEquals(-3.115, servo.getWrappedPositionRad(), 0.001);
        assertEquals(-3.115, servo.getUnwrappedPositionRad(), 0.001);
        for (int i = 17; i < 40; ++i) {
            servo.setPositionProfiled(3, 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // now the wrapped angle is what we asked for
        assertEquals(3, servo.getWrappedPositionRad(), 0.001);
        // and the unwrapped one shows that we went past -pi
        assertEquals(-3.283, servo.getUnwrappedPositionRad(), 0.001);
    }

    /**
     * This takes the long way around due to the mechanism limits, using profiled
     * motion.
     */
    @Test
    void testLongWayOnboardProfiled() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, -3.1, 3.1);
        Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        IncrementalProfile profile = new TrapezoidIncrementalProfile(2, 2, 0.05);
        // IncrementalProfile profile = new TrapezoidProfileWPI(2, 2);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);

        // at zero
        servo.reset();
        servo.periodic();
        stepTime();

        // move to the starting point of -3
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(-3, -150)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-1.5, -150), new Control100(-3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // start motionless at -3
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-3, servo.getWrappedPositionRad(), DELTA);

        // move to 3
        for (int i = 0; i < 100; ++i) {
            servo.setPositionProfiled(3, 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f velocity: %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // passing through zero
        assertEquals(0, servo.getWrappedPositionRad(), 0.001);
        assertEquals(0, servo.getUnwrappedPositionRad(), 0.001);
        for (int i = 100; i < 250; ++i) {
            servo.setPositionProfiled(3, 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // now the wrapped angle is what we asked for
        assertEquals(3, servo.getWrappedPositionRad(), 0.001);
        assertEquals(3, servo.getUnwrappedPositionRad(), 0.001);
    }

    @Test
    void testDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 50)), 0);
        stepTime();

        // this is setpoint velocity
        assertEquals(50, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(50, encoder.getVelocityRad_S(), DELTA);
        assertEquals(50, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getWrappedPositionRad(), DELTA);

        servo.periodic();
        // note position to match the integrator
        servo.setPositionDirect(new Setpoints1d(new Control100(0.5, 50), new Control100(1, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        // all the way there now
        assertEquals(1, sensor.getWrappedPositionRad(), DELTA);
    }

    /** From -3 to 3 the short way */
    @Test
    void testShortWayOnboardDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        // lots of feedback here since there's no setpoint velocity.
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 10, 0, 0, false, 0.05, 1);
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(-3, -150)), 0);
        stepTime();

        // this is setpoint velocity
        assertEquals(-150, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-150, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-150, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-1.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-1.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        // note position to match the integrator
        servo.setPositionDirect(new Setpoints1d(new Control100(-1.5, -150), new Control100(-3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-3, servo.getWrappedPositionRad(), DELTA);

        // now try to go to 3, the "short way"
        // since this relies (in this test) on feedback alone, the shape is
        // exponential.

        for (int i = 0; i < 4; ++i) {
            // use the goal as both setpoints.
            servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // wrapped angle has crossed over
        assertEquals(3.120, servo.getWrappedPositionRad(), 0.001);
        // unwrapped continues
        assertEquals(-3.163, servo.getUnwrappedPositionRad(), 0.001);
        for (int i = 4; i < 20; ++i) {
            // use the goal as both setpoints.
            servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // feedback overshoots a little
        assertEquals(3.002, servo.getWrappedPositionRad(), 0.001);
        assertEquals(-3.282, servo.getUnwrappedPositionRad(), 0.001);
    }

    /** From -3 to 3 the long way with "direct". */
    @Test
    void testLongWayOnboardDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, -3.1, 3.1);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        // lots of feedback here since control velocity is zero
        // note that we don't use "continuous" feedback here
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 5, 0, 0, false, 0.05, 1);
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);

        servo.reset();
        servo.periodic();
        stepTime();

        // move to the starting point of -3
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(-3, -150)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-1.5, -150), new Control100(-3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // start motionless at -3
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-3, servo.getWrappedPositionRad(), DELTA);

        // now try to go to 3
        if (DEBUG)
            Util.println("** now try to go to 3, the long way");
        // since this relies (in this test) on feedback alone, the shape is
        // exponential.

        for (int i = 0; i < 5; ++i) {
            // use the goal as both setpoints.
            servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // angle crosses zero
        assertEquals(-0.657, servo.getWrappedPositionRad(), 0.001);
        assertEquals(-0.657, servo.getUnwrappedPositionRad(), 0.001);
        for (int i = 4; i < 50; ++i) {
            // use the goal as both setpoints.
            servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
            servo.periodic();
            stepTime();
            if (DEBUG)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
        // it takes a long time to get all the way there, since it's only proportional
        // feedback
        assertEquals(2.979, servo.getWrappedPositionRad(), 0.001);
        assertEquals(2.979, servo.getUnwrappedPositionRad(), 0.001);

    }

}
