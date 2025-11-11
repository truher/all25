package org.team100.lib.dynamics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RRDynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // straight up
        RRTorque t = d.torque(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // no torques
        assertEquals(0, t.f1(), DELTA);
        assertEquals(0, t.t2(), DELTA);
    }

    @Test
    void test1() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // to the side
        RRTorque t = d.torque(
                new RRConfig(Math.PI / 2, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg is 0.5 m away, so 5Nm, 1 kg 1.5 m away so 15Nm
        assertEquals(-19.6, t.f1(), DELTA);
        // 1 kg 0.5 m away
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test2() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // wrist only to the side (bent arm)
        RRTorque t = d.torque(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.f1(), DELTA);
        // 1 kg 0.5 m away (same as above)
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test3() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm moving at the root
        RRTorque t = d.torque(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.f1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-4.4, t.t2(), DELTA);
    }

    @Test
    void test4() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm accelerating at the root
        RRTorque t = d.torque(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(1, 0));
        // 1 kg 0.5 m away so 5Nm, minus centrifugal force
        assertEquals(-1.4, t.f1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-3.65, t.t2(), DELTA);
    }

    @Test
    void test5() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // like a whip: extended, moving, slowing down at the root
        RRTorque t = d.torque(
                new RRConfig(0, 0),
                new RRVelocity(1, 0),
                new RRAcceleration(-1, 0));
        // elbow tries to keep going, so push back
        assertEquals(-4.5, t.f1(), DELTA);
        // trying to slow down
        assertEquals(-1.75, t.t2(), DELTA);
    }

}
