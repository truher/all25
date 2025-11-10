package org.team100.lib.kinematics.urdf;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public record URDFJoint(
        String name,
        JointType type,
        Limit limit,
        URDFLink parent,
        URDFLink child,
        Pose3d origin,
        Vector<N3> axis) {
    public record Limit(double effort, double lower, double upper, double velocity) {
    }

    public enum JointType {
        revolute,
        continuous,
        prismatic,
        fixed,
        floating,
        planar
    }

    boolean active() {
        // skip fixed since it doesn't have a parameter and thus breaks the solver
        return type() == JointType.revolute
                || type() == JointType.continuous
                || type() == JointType.prismatic;
    }

    /**
     * Transform for a single joint.
     * The parameter, q, can be null, for fixed joints.
     * First transform is the "origin" transform (in the parent frame), followed by
     * the joint transform (rotation or translation).
     */
    Transform3d transform(Double q) {
        // First, translate along the link, in the parent frame.
        Transform3d linkTransform = new Transform3d(Pose3d.kZero, origin());

        // Then, rotate or translate as appropriate.
        Transform3d jointTransform = switch (type()) {
            case revolute, continuous -> new Transform3d(0, 0, 0, new Rotation3d(axis(), q));
            case prismatic -> new Transform3d(new Translation3d(axis().times(q)), Rotation3d.kZero);
            case fixed -> Transform3d.kZero;
            default -> throw new UnsupportedOperationException();
        };

        return linkTransform.plus(jointTransform);
    }
}