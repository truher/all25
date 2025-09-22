package org.team100.dynamics.pr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        Dynamics d = new Dynamics(1, 1, 1, 1);
        // configuration is facing upwards
        Torque t = d.torque(
                new Config(0, 0),
                new Velocity(0, 0),
                new Acceleration(0, 0));
        // total mass is 2kg, gravity is 9.8, so -x direction,
        assertEquals(-19.6, t.f1(), DELTA);
        // arm facing up means no torque
        assertEquals(0, t.t2(), DELTA);
    }

    @Test
    void test1() {
        Dynamics d = new Dynamics(1, 1, 1, 1);
        // configuration is arm to the side
        // motionless
        Torque t = d.torque(
                new Config(0, Math.PI / 2),
                new Velocity(0, 0),
                new Acceleration(0, 0));
        // total mass is 2kg, gravity is 9.8
        assertEquals(-19.6, t.f1(), DELTA);
        // arm out, 1m lever, 1kg, so 9.8 Nm, the direction is CCW
        assertEquals(9.8, t.t2(), DELTA);
    }

    @Test
    void test2() {
        Dynamics d = new Dynamics(1, 1, 1, 1);
        // configuration is arm to the side
        //  elevator accelerating up
        Torque t = d.torque(
                new Config(0, Math.PI / 2),
                new Velocity(0, 0),
                new Acceleration(1, 0));
        // total mass is 2kg, gravity is 9.8
        assertEquals(-19.6, t.f1(), DELTA);
        // 
        assertEquals(0, t.t2(), DELTA);
    }

}
