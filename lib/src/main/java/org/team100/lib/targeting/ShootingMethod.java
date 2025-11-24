package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.optimization.NewtonsMethod;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/**
 * Solve the intercept problem via the "shooting method" for fixed muzzle
 * velocity and spin.
 */
public class ShootingMethod {
    public record Solution(Rotation2d azimuth, Rotation2d elevation) {
    }

    private final Range m_range;
    private final double m_tolerance;

    public ShootingMethod(Range range, double tolerance) {
        m_range = range;
        m_tolerance = tolerance;
    }

    public Optional<Solution> solve(
            Translation2d robotPosition,
            GlobalVelocityR2 robotVelocity,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        // target relative to robot
        Translation2d T0 = targetPosition.minus(robotPosition);
        // target velocity relative to robot
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);

        // domain is (az,el), range is error
        Vector<N2> xMin = VecBuilder.fill(-Math.PI, 0);
        Vector<N2> xMax = VecBuilder.fill(Math.PI, Math.PI / 2);
        int iterations = 10;
        double dxLimit = 0.1;
        NewtonsMethod<N2, N2> solver = new NewtonsMethod<>(
                Nat.N2(),
                Nat.N2(),
                fn(T0, vT),
                xMin,
                xMax,
                m_tolerance,
                iterations,
                dxLimit);
        try {
            Vector<N2> x = solver.solve2(VecBuilder.fill(0, 0), 3, true);
            return Optional.of(
                    new Solution(
                            new Rotation2d(x.get(0)),
                            new Rotation2d(x.get(1))));
        } catch (IllegalArgumentException ex) {
            return Optional.empty();
        }
    }

    /** Curry */
    Function<Vector<N2>, Vector<N2>> fn(Translation2d T0, GlobalVelocityR2 vT) {
        return x -> this.f(x, T0, vT);
    }

    Vector<N2> f(Vector<N2> x, Translation2d T0, GlobalVelocityR2 vT) {
        // Extract contents of the state variable
        Rotation2d azimuth = new Rotation2d(x.get(0));
        double elevation = x.get(1);
        // Lookup for this state.
        Range.Solution rangeSolution = m_range.get(elevation);
        // Ball location at impact, relative to initial robot position
        Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
        // target location at impact, relative to initial robot position
        Translation2d T = vT.integrate(T0, rangeSolution.tof());
        Translation2d err = b.minus(T);
        // result error is (x, y)
        return GeometryUtil.toVec(err);
    }

}
