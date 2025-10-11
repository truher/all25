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
    private static final boolean ACTUALLY_PRINT = true;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testOnboard() {
        final MockBareMotor turningMotor = new MockBareMotor(Feedforward100.makeSimple());
        final MockRotaryPositionSensor positionSensor = new MockRotaryPositionSensor();
        final RotaryMechanism mech = new RotaryMechanism(
                logger, turningMotor, positionSensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger, mech, ref, turningFeedback2);
        servo.reset();
        // spin for 1 s
        for (int i = 0; i < 50; ++i) {
            servo.setPositionProfiled(1, 0);
            stepTime();
            if (ACTUALLY_PRINT)
                Util.printf("i: %d position: %5.3f %5.3f\n", i, turningMotor.position, turningMotor.velocity);
            // lets say we're on the profile.
            positionSensor.angle = servo.m_unwrappedSetpoint.x();
            positionSensor.rate = servo.m_unwrappedSetpoint.v();
        }
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.m_unwrappedSetpoint.x(), DELTA);
        assertEquals(1.0, servo.m_unwrappedSetpoint.v(), DELTA);
        assertEquals(0.5, positionSensor.getWrappedPositionRad(), DELTA);
        assertEquals(1.000, turningMotor.velocity, DELTA);
    }

    /** This should take the short path. */
    @Test
    void testWrappedOnboard() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
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
        assertEquals(-3, servo.getPosition(), DELTA);

        // move to 3
        for (int i = 0; i < 200; ++i) {
            servo.setPositionProfiled(3, 0);
            stepTime();
            if (ACTUALLY_PRINT)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
    }

    /** This takes the long way around. */
    @Test
    void testUnrappedOnboard() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
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
        assertEquals(-3, servo.getPosition(), DELTA);

        // move to 3
        for (int i = 0; i < 200; ++i) {
            servo.setPositionProfiled(3, 0);
            stepTime();
            if (ACTUALLY_PRINT)
                Util.printf("i: %d position: %5.3f %5.3f\n",
                        i, motor.getUnwrappedPositionRad(), motor.getVelocityRad_S());
        }
    }

    @Test
    void testDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
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
    void testDirectWrapped() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
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
        assertEquals(0, servo.getPosition(), DELTA);

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
        assertEquals(-1.5, servo.getPosition(), DELTA);

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
        assertEquals(-3, servo.getPosition(), DELTA);

        // now try to go to 3, the "short way"

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(3, -14.159)), 0);
        stepTime();

        // this is setpoint velocity
        assertEquals(-14.159, motor.getVelocityRad_S(), DELTA);
        // encoder is unwrapped
        assertEquals(-3.283, encoder.getUnwrappedPositionRad(), 0.001);
        assertEquals(-14.159, encoder.getVelocityRad_S(), 0.001);
        assertEquals(-14.159, mech.getVelocityRad_S(), 0.001);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-3.14159, sensor.getWrappedPositionRad(), 0.001);
        assertEquals(-3.14159, servo.getPosition(), 0.001);

        servo.periodic();
        // note position to match the integrator
        servo.setPositionDirect(new Setpoints1d(new Control100(-3.14159, -14.159), new Control100(3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), 0.001);
        assertEquals(-3.283, encoder.getUnwrappedPositionRad(), 0.001);
        assertEquals(0, encoder.getVelocityRad_S(), 0.001);
        assertEquals(0, mech.getVelocityRad_S(), 0.001);
        assertEquals(3, sensor.getWrappedPositionRad(), 0.001);
        assertEquals(3, servo.getPosition(), 0.001);
    }

    /** From -3 to 3 the long way */
    // TODO: fix this
    // @Test
    void testDirectUnwrapped() {
        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(logger, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, true, 0.05, 1);
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
        assertEquals(0, servo.getPosition(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(-3, 150)), 0);
        stepTime();

        // this is setpoint velocity
        assertEquals(-150, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-150, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-150, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-1.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-1.5, servo.getPosition(), DELTA);

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
        assertEquals(-3, servo.getPosition(), DELTA);

        // now try to go to 3, the "short way"

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(3, -14.159)), 0);
        stepTime();

        // this is setpoint velocity
        assertEquals(-14.159, motor.getVelocityRad_S(), DELTA);
        // encoder is unwrapped
        assertEquals(-3.283, encoder.getUnwrappedPositionRad(), 0.001);
        assertEquals(-14.159, encoder.getVelocityRad_S(), 0.001);
        assertEquals(-14.159, mech.getVelocityRad_S(), 0.001);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-3.14159, sensor.getWrappedPositionRad(), 0.001);
        assertEquals(-3.14159, servo.getPosition(), 0.001);

        servo.periodic();
        // note position to match the integrator
        servo.setPositionDirect(new Setpoints1d(new Control100(-3.14159, -14.159), new Control100(3, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), 0.001);
        assertEquals(-3.283, encoder.getUnwrappedPositionRad(), 0.001);
        assertEquals(0, encoder.getVelocityRad_S(), 0.001);
        assertEquals(0, mech.getVelocityRad_S(), 0.001);
        assertEquals(3, sensor.getWrappedPositionRad(), 0.001);
        assertEquals(3, servo.getPosition(), 0.001);
    }

}
