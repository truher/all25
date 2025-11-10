package org.team100.lib.dynamics.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class PRRDynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // configuration is facing upwards
        PRRTorque t = d.torque(
                new PRRConfig(0, 0, 0),
                new PRRVelocity(0, 0, 0),
                new PRRAcceleration(0, 0, 0));
        // total mass is 3kg, gravity is 9.8, so force to oppose gravity
        // is upwards, 29.4
        assertEquals(29.4, t.f1(), DELTA);
        // arm facing up means no torque
        assertEquals(0, t.t2(), DELTA);
        assertEquals(0, t.t3(), DELTA);
    }

    @Test
    void test1() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // configuration is arm to the side at the shoulder
        // motionless
        PRRTorque t = d.torque(
                new PRRConfig(0, Math.PI / 2, 0),
                new PRRVelocity(0, 0, 0),
                new PRRAcceleration(0, 0, 0));
        // 3 kg same total mass, no movement
        assertEquals(29.4, t.f1(), DELTA);
        // 1kg at 0.5m: 5 Nm + 1kg at 1.5m: 15 Nm = 20.
        // arm out, 1m lever, 1kg, so gravity is 9.8 Nm, the torque to oppose
        // is CW (negative)
        assertEquals(-19.6, t.t2(), DELTA);
        // 1kg at 0.5 relative to the joint, so 5 Nm.
        assertEquals(-4.9, t.t3(), DELTA);
    }

    @Test
    void test2() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // configuration is arm to the side at the shoulder
        // elevator accelerating up
        PRRTorque t = d.torque(
                new PRRConfig(0, Math.PI / 2, 0),
                new PRRVelocity(0, 0, 0),
                new PRRAcceleration(1, 0, 0));
        // upward accel is like extra gravity, 3kg 1 m/s^2 means an extra 3N
        assertEquals(32.4, t.f1(), DELTA);
        // upward accel is like extra gravity, 0.5 for l2, 1.5 for l3, so +2 Nm
        assertEquals(-21.6, t.t2(), DELTA);
        // upward accel is like extra gravity, 0.5
        assertEquals(-5.4, t.t3(), DELTA);
    }

    @Test
    void test3() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent at the wrist, motionless
        PRRTorque t = d.torque(
                new PRRConfig(0, 0, Math.PI / 2),
                new PRRVelocity(0, 0, 0),
                new PRRAcceleration(0, 0, 0));
        // total weight
        assertEquals(29.4, t.f1(), DELTA);
        // wrist gravity torque is same at shoulder
        assertEquals(-4.9, t.t2(), DELTA);
        // wrist gravity torque
        assertEquals(-4.9, t.t3(), DELTA);
    }

    @Test
    void test4() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent at the wrist, same as above, but moving at the shoulder,
        // this shows centrifugal force on arm and hand.
        PRRTorque t = d.torque(
                new PRRConfig(0, 0, Math.PI / 2),
                new PRRVelocity(0, 1, 0),
                new PRRAcceleration(0, 0, 0));
        // centrifugal force helps against gravity, so less than above
        assertEquals(27.9, t.f1(), DELTA);
        // gravity on the hand, effect on shoulder, motion doesn't matter
        assertEquals(-4.9, t.t2(), DELTA);
        // centrifual force on the hand, effect on wrist
        assertEquals(-4.4, t.t3(), DELTA);
    }

    @Test
    void test5() {
        PRRDynamics d = new PRRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent at the wrist, same as above, but accel at the shoulder,
        // no motion.  not sure this is right.
        PRRTorque t = d.torque(
                new PRRConfig(0, 0, Math.PI / 2),
                new PRRVelocity(0, 0, 0),
                new PRRAcceleration(0, 1, 0));
        // less than motionless
        assertEquals(28.9, t.f1(), DELTA);
        // a lot less than motionless
        assertEquals(-1.4, t.t2(), DELTA);
        // less than motionless
        assertEquals(-3.65, t.t3(), DELTA);
    }

}
