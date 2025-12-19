package org.team100.lib.trajectory.timing;

import org.junit.jupiter.api.Test;

public class MultipleShootingTest {
    @Test
    void test0() {
        // x is fixed
        double[] x = new double[] { 1, 2, 2 };
        // v endpoints are fixed
        double[] v = new double[] { 0, 0, 0 };
        // a is between, has no fixed values
        Double[] a = new Double[] { null, null };
        double maxV = 1;
        double maxA = 1;
        MultipleShooting shooter = new MultipleShooting(maxV, maxA);
        shooter.solve(x, v, a);
    }

    @Test
    void test1() {
        // x is fixed
        double[] x = new double[] { 1, 2, 3 };
        // v endpoints are fixed
        double[] v = new double[] { 0, 0, 0 };
        // a is between, has no fixed values
        Double[] a = new Double[] { null, null };
        double maxV = 1;
        double maxA = 1;
        MultipleShooting shooter = new MultipleShooting(maxV, maxA);
        shooter.solve(x, v, a);
    }

}
