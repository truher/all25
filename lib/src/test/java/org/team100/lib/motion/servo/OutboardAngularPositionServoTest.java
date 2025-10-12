package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
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
import org.team100.lib.reference.MockProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OutboardAngularPositionServoTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final boolean ACTUALLY_PRINT = false;

    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testProfiled() {
        final MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());
        final MockIncrementalBareEncoder encoder = new MockIncrementalBareEncoder();
        final MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();

        final ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        final CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                log, sensor, proxy);

        final RotaryMechanism mech = new RotaryMechanism(
                log, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.01, 0.01);
        final OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);
        servo.reset();
        // it moves slowly
        servo.setPositionProfiled(1, 0);
        stepTime();

        assertEquals(2e-4, motor.position, 1e-4);
        servo.setPositionProfiled(1, 0);
        stepTime();

        // assertEquals(8e-4, motor.position, 1e-4);
        servo.setPositionProfiled(1, 0);
        stepTime();

        assertEquals(0.002, motor.position, DELTA);
        for (int i = 0; i < 100; ++i) {
            // run it for awhile
            servo.setPositionProfiled(1, 0);
            stepTime();
            if (ACTUALLY_PRINT)
                Util.printf("i: %d position: %5.3f\n", i, motor.position);
        }
        assertEquals(1, motor.position, DELTA);
    }

    /** Within +/- pi, no surprises. */
    @Test
    void testDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        // no profile for this test.
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        // move 0 to 1 in 0.02 => v = 50
        assertEquals(50, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(50, encoder.getVelocityRad_S(), DELTA);
        assertEquals(50, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(1, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(1, servo.getWrappedPositionRad(), DELTA);
    }

    @Test
    void testMod() {
        MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        // no profile for this test.
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);
        // 0 -> 3
        assertEquals(0, mech.getWrappedPositionRad(), DELTA);
        assertEquals(3, servo.mod(3), DELTA);
        // -3 -> 3 the short way around
        sensor.angle = -3;
        assertEquals(-3, mech.getWrappedPositionRad(), DELTA);
        assertEquals(-3.283, servo.mod(3), DELTA);
    }

    /**
     * A multiturn mechanism might be something like a turret: it can move more than
     * one turn, but not infinity turns, and within its range of motion, wrapped
     * angles are equivalent (i.e. what matters is where the turret is pointing). So
     * in this case, we should use the "short way around" within the range of
     * motion, but the "long way around" for goals outside the limit.
     */
    // TODO: fix this
    @Test
    void testDirectMultiturn() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        // total range is 4.5 turns
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, -9.0 * Math.PI / 4.0, 9.0 * Math.PI / 4.0);
        // no profile for this test.
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

        // Start at zero.

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);

        System.out.println("Move a quarter turn in the positive direction");

        servo.periodic();
        Control100 ignored = new Control100(0, 0);
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(Math.PI / 2, 0)), 0);
        stepTime();

        // +v
        assertEquals(78.540, motor.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 2, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(78.540, encoder.getVelocityRad_S(), DELTA);
        assertEquals(78.540, mech.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 4, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 4, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(Math.PI / 2, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 2, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 2, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 2, servo.getWrappedPositionRad(), DELTA);

        System.out.println("Try to go one turn away directly? That does nothing.");

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(5.0 * Math.PI / 2, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 2, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(Math.PI / 2, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(Math.PI / 2, servo.getWrappedPositionRad(), DELTA);

        System.out.println("move towards the limit a little at a time");

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(Math.PI, 0)), 0);
        stepTime();
        servo.periodic();
        stepTime();
        assertEquals(Math.PI, motor.getUnwrappedPositionRad(), DELTA);
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(3 * Math.PI / 2, 0)), 0);
        stepTime();
        servo.periodic();
        stepTime();
        assertEquals(3 * Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(2 * Math.PI, 0)), 0);
        stepTime();
        servo.periodic();
        stepTime();
        assertEquals(2 * Math.PI, motor.getUnwrappedPositionRad(), DELTA);
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(5 * Math.PI / 2, 0)), 0);
        stepTime();
        servo.periodic();
        stepTime();
        assertEquals(5 * Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        // again so the integrator catches up
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(ignored, new Control100(5.0 * Math.PI / 2, 0)), 0);
        stepTime();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(5 * Math.PI / 2, motor.getUnwrappedPositionRad(), DELTA);
        assertEquals(5 * Math.PI / 2, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(5 * Math.PI / 2, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(5 * Math.PI / 2, servo.getWrappedPositionRad(), DELTA);
    }

    /**
     * This is a mechanism that can turn infinitely, and where only direction
     * matters, e.g. the swerve azimuth axis. We should always go the "short way
     * around".
     */
    @Test
    void testDirectContinuous() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

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
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        // move 0 to 1 in 0.02 => v = 50
        assertEquals(50, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(50, encoder.getVelocityRad_S(), DELTA);
        assertEquals(50, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(1, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(1, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(1, servo.getWrappedPositionRad(), DELTA);
    }

    /**
     * Let's say we have a mechanism that we want to control in an "unwrapped" way,
     * i.e. if the measurement is, say, -3, and we want to go to, say, 3, we really
     * want to go the "long way around". This would come up in the case of a
     * mechanism with a physical limit somewhere -- you can't just choose a
     * direction because it's "nearby". Or it could be a multi-turn mechanism, e.g.
     * a turret that can travel 1.5 turns or something.
     */
    // TODO: fix this
    // @Test
    void testDirectUnwrapped() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);

        // First go to -3.
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // back up 3 in 0.02, so v=-150.
        assertEquals(-150, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-150, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-150, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-1.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-1.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);

        // Now try to go to 3. We want the "long way around."
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        assertEquals(-150, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-150, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-150, mech.getVelocityRad_S(), DELTA);
        assertEquals(-1.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-1.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
    }

    /**
     * Like above but wrapped -- take the "short way."
     */
    // TODO: fix this
    // @Test
    void testDirectWrapped() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(0, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);

        // First go to -3.
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // back up 3 in 0.02, so v=-150.
        assertEquals(-150, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-150, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-150, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(-1.5, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(-1.5, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(-3, 0), new Control100(-3, 0)), 0);
        stepTime();

        // all the way there now
        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);

        // Now try to go to 3. We want the "short way around."
        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        // to get from -3 to 3 the short way, we go in the *negative* direction.
        assertEquals(-14.159, motor.getVelocityRad_S(), DELTA);
        // encoder is unwrapped
        assertEquals(-3.283, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(-14.159, encoder.getVelocityRad_S(), DELTA);
        assertEquals(-14.159, mech.getVelocityRad_S(), DELTA);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(3.142, sensor.getWrappedPositionRad(), DELTA);
        assertEquals(3.142, servo.getWrappedPositionRad(), DELTA);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(3, 0), new Control100(3, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), DELTA);
        assertEquals(-3, encoder.getUnwrappedPositionRad(), DELTA);
        assertEquals(0, encoder.getVelocityRad_S(), DELTA);
        assertEquals(0, mech.getVelocityRad_S(), DELTA);
        // all the way there now
        assertEquals(-3, sensor.getWrappedPositionRad(), DELTA);
    }
}
