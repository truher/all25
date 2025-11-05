package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

/** Uses a lookup table to find range from elevation, for fixed muzzle speed. */
public class Range {
    public record Solution(double range, double tof) {
    }

    public class SolutionInterpolator implements Interpolator<Solution> {

        @Override
        public Solution interpolate(Solution a, Solution b, double t) {
            return new Solution(
                    MathUtil.interpolate(a.range, b.range, t), MathUtil.interpolate(a.tof, b.tof, t));
        }
    }

    private static final boolean DEBUG = false;
    private final InterpolatingTreeMap<Double, Solution> m_map;

    /**
     * @param d     drag model
     * @param v     muzzle speed
     * @param omega spin, positive is backspin
     */
    public Range(Drag d, double v, double omega) {
        m_map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new SolutionInterpolator());
        double dt = 0.001;

        if (DEBUG)
            System.out.println("elevation, range, tof");
        for (double elevation = 0; elevation < Math.PI / 2; elevation += 0.01) {
            double vx = v * Math.cos(elevation);
            double vy = v * Math.sin(elevation);
            Matrix<N6, N1> x = VecBuilder.fill(0, 0, 0, vx, vy, omega);
            double t = 0;
            for (t = 0; t < 10; t += dt) {
                x = NumericalIntegration.rk4(d, x, dt);
                if (x.get(1, 0) < 0)
                    break;
            }
            double range = x.get(0, 0);
            double tof = t;
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n", elevation, range, tof);
            m_map.put(elevation, new Solution(range, tof));
        }
    }

    public Solution get(double elevation) {
        return m_map.get(elevation);
    }

}
