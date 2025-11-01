package org.team100.lib.targeting;

import java.util.function.UnaryOperator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/**
 * Newton drag is proportional to the square of velocity.
 * 
 * Drag force is -0.5 * c * rho * A * v^2
 * 
 * This does not include the Magnus effect, so it's not good for spinning balls,
 * but it's ok for "knuckleballs".
 * 
 * This uses notation from here:
 * 
 * https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
 */
public class NewtonDrag implements UnaryOperator<Matrix<N4, N1>> {
    /** Air density, kg/m^3 */
    private static final double RHO = 1.225;
    /** Gravity, m/s^2 */
    private static final double G = 9.81;

    private final double mu;

    /**
     * @param c drag coefficient
     * @param A area, m^2
     * @param m mass, kg
     */
    public NewtonDrag(double c, double A, double m) {
        mu = c * RHO * A / (2 * m);
    }

    /**
     * The time derivative of state.
     * 
     * @param x the current state: (x, y, vx, vy)
     */
    public Matrix<N4, N1> apply(Matrix<N4, N1> x) {
        double vx = x.get(2, 0);
        double vy = x.get(3, 0);
        double v = Math.sqrt(vx * vx + vy * vy);
        double ax = -mu * vx * v;
        double ay = -G - mu * vy * v;
        return VecBuilder.fill(vx, vy, ax, ay);
    }
}
