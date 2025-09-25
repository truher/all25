package org.team100.frc2025.CalgamesArm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** use the jacobian to find these from a cartesian velocity. */
public record JointVelocities(double elevator, double shoulder, double wrist) {
    public static JointVelocities fromVector(Vector<N3> v) {
        return new JointVelocities(v.get(0), v.get(1), v.get(2));
    }

    public static JointVelocities fromVector(Matrix<N3, N1> v) {
        return new JointVelocities(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(elevator, shoulder, wrist);
    }
}
