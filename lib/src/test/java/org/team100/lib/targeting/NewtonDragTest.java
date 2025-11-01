package org.team100.lib.targeting;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

public class NewtonDragTest {
    /**
     * https://docs.google.com/spreadsheets/d/11GQr9xW8jVH54ecrWKZy7nI1lZMZq_tu9DBvIxDnySA/edit?gid=0#gid=0
     */
    @Test
    void testIntegration() {
        NewtonDrag f = new NewtonDrag(0.5, 0.1, 0.1);
        Matrix<N4, N1> x = VecBuilder.fill(0, 0, 5, 5);
        double dt = 0.005;
        System.out.println("t, x, y");
        for (double t = 0; t < 10; t += dt) {
            System.out.printf("%7.4f, %7.4f, %7.4f\n", t, x.get(0, 0), x.get(1, 0));
            x = NumericalIntegration.rk4(f, x, dt);
            if (x.get(1, 0) < 0)
                break;
        }
    }

}
