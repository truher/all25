package org.team100.lib.dynamics.r;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RDynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        RDynamics d = new RDynamics(1, 1, 0.5, 1);
        // straight up
        RTorque t = d.torque(
                new RConfig(0),
                new RVelocity(0),
                new RAcceleration(0));
        // no torque
        assertEquals(0, t.f1(), DELTA);
    }

    @Test
    void test1() {
        RDynamics d = new RDynamics(1, 1, 0.5, 1);
        // to the side
        RTorque t = d.torque(
                new RConfig(Math.PI / 2),
                new RVelocity(0),
                new RAcceleration(0));
        // 1 kg is 0.5 m away, so 5Nm
        assertEquals(-4.9, t.f1(), DELTA);

    }


    @Test
    void test3() {
        RDynamics d = new RDynamics(1, 1, 0.5, 1);
        // moving
        RTorque t = d.torque(
                new RConfig(0),
                new RVelocity(1),
                new RAcceleration(0));
        // no gravity force
        assertEquals(0, t.f1(), DELTA);

    }

    @Test
    void test4() {
        RDynamics d = new RDynamics(1, 1, 0.5, 1);
        //  accelerating 
        RTorque t = d.torque(
                new RConfig(0),
                new RVelocity(0),
                new RAcceleration(1));
        // mass torque 0.25 + inertia 1 = 1.25
        assertEquals(1.25, t.f1(), DELTA);

    }

    @Test
    void test5() {
        RDynamics d = new RDynamics(1, 1,0.5, 1);
        //  slowing down 
        RTorque t = d.torque(
                new RConfig(0),
                new RVelocity(1),
                new RAcceleration(-1));
        // slowing is the same, velocity doesn't matter
        assertEquals(-1.25, t.f1(), DELTA);

    }

}
