package org.team100.lib.motion.urdf;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.math.NewtonsMethod;
import org.team100.lib.motion.urdf.URDFModel.Joint.JointType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;

/**
 * This is a partial implementation of the URDF object model.
 * 
 * The idea is to use a more general mechanism representation than the usual
 * POJO (e.g. LynxArm, which uses fields), which would allow more general
 * kinematics than the hand-made stuff we've done previously (e.g. we could try
 * Cyclic Coordinate Descent, or Gradient Descent, in a general way)
 * 
 * It would be possible to serialize this to URDF XML and vice versa, but our
 * robots are small enough that (IMHO) putting the "loader" in code would be
 * much easier than bringing in the XML overhead.
 * 
 * See http://wiki.ros.org/urdf, in particular, urdf.xsd. I transcribed directly
 * from that file. (I tried to use JAXB to auto-generate this, but gave up
 * trying to run it.)
 * 
 * This omits the collision and visual parts of the model, which are used for
 * simulation and visualization, respectively. We could add those back in if we
 * need them someday. I haven't found the URDF visualizers (e.g.
 * https://mymodelrobot.appspot.com/5629499534213120) or simulators (e.g.
 * Gazebo) to be that useful.
 * 
 * TODO: normalize the "axis" vector or complain if not normalized.
 * 
 * There exists SRDF, which specifies, among other things, group states, e.g.
 * home positions. I think SRDF is dead, but representing states seems fine;
 * the format is a map of joint names and values.
 * 
 * https://wiki.ros.org/srdf/review
 */
public interface URDFModel {

    

    public record Link(String name) {
    }

    public record Joint(
            String name,
            JointType type,
            Limit limit,
            Link parent,
            Link child,
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

        /**
         * Transform for a single joint.
         * The parameter, q, can be null, for fixed joints.
         * First transform is the "origin" transform (in the parent frame), followed by
         * the joint transform (rotation or translation).
         */
        public Transform3d jointTransform(Double q) {
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

    public record Robot(String name, List<Link> links, List<Joint> joints) {

        /** Name-keyed joint poses from name-keyed q values. */
        public Map<String, Pose3d> forward(Map<String, Double> qMap) {
            Map<String, Pose3d> poses = new HashMap<>();
            for (URDFModel.Joint joint : joints()) {
                forward(poses, joint.name(), qMap);
            }
            return poses;
        }

        /**
         * Populate poses with the pose of the specified joint and those of its parent
         * chain.
         */
        public Pose3d forward(
                Map<String, Pose3d> poses,
                String jointName,
                Map<String, Double> qMap) {
            Joint joint = getJoint(jointName);
            Transform3d t = joint.jointTransform(qMap.get(jointName));
            Joint parent = parentJoint(jointName);
            if (parent == null) {
                // this is the root
                Pose3d basePose = Pose3d.kZero.transformBy(t);
                poses.put(jointName, basePose);
                return basePose;
            }
            Pose3d parentPose = poses.get(parent.name());
            if (parentPose != null) {
                Pose3d newPose = parentPose.transformBy(t);
                poses.put(jointName, newPose);
                return newPose;
            }
            // if the parent hasn't been computed yet, recurse to do it.
            Pose3d newParentPose = forward(poses, parent.name(), qMap);
            Pose3d newNewPose = newParentPose.transformBy(t);
            poses.put(jointName, newNewPose);
            return newNewPose;
        }

        /**
         * Solve inverse kinematics for all joints using Newton's method.
         * 
         * qDim indicates the dimensionality of the configuration space.
         * TODO: get rid of qDim.
         * q0 is the initial (e.g. current) configuration.
         */
        public <Q extends Num> Map<String, Double> inverse(
                Nat<Q> qDim,
                Vector<Q> q0,
                double dqLimit,
                String jointName,
                Pose3d goal) {
            Function<Vector<Q>, Vector<N6>> f = q -> {
                // print("q", q);
                // solve all of them
                Map<String, Double> qMap = qMap(q);
                Map<String, Pose3d> p = forward(qMap);
                // Pick out the joint we want.
                Pose3d pose = p.get(jointName);
                Vector<N6> pv = GeometryUtil.toVec(GeometryUtil.slog(pose));
                // print(pose);
                // print("pv", pv);
                return pv;
            };
            // this function always uses pose3d so the goal dim is always N6.
            Nat<N6> twistDim = Nat.N6();
            Vector<N6> goalVec = GeometryUtil.toVec(GeometryUtil.slog(goal));
            NewtonsMethod<Q, N6> solver = new NewtonsMethod<>(
                    qDim, twistDim, f,
                    minQ(qDim), maxQ(qDim),
                    1e-3, 20, dqLimit);
            Vector<Q> q = solver.solve2(q0, goalVec);
            return qMap(q);
        }

        public Joint getJoint(String name) {
            for (Joint joint : joints()) {
                if (joint.name().equals(name))
                    return joint;
            }
            return null;
        }

        /** parent joint of the specified joint */
        public Joint parentJoint(String childName) {
            URDFModel.Joint childJoint = getJoint(childName);
            URDFModel.Link parentLink = childJoint.parent();
            for (URDFModel.Joint joint : joints()) {
                if (joint.child() == parentLink)
                    return joint;
            }
            return null;
        }

        /** Transform the config vector, q, into a named map. */
        public Map<String, Double> qMap(Vector<?> q) {
            Map<String, Double> qMap = new HashMap<>();
            List<URDFModel.Joint> joints = joints();
            for (int i = 0; i < joints.size(); ++i) {
                URDFModel.Joint joint = joints.get(i);
                if (active(joint)) {
                    qMap.put(joint.name(), q.get(i));
                }
            }
            return qMap;
        }

        /**
         * qDim needs to match the actual number of moveable joints.
         * TODO: remove it.
         */
        public <Q extends Num> Vector<Q> minQ(Nat<Q> qDim) {
            Vector<Q> v = new Vector<>(qDim);
            List<URDFModel.Joint> joints = joints();
            for (int i = 0; i < joints.size(); ++i) {
                URDFModel.Joint joint = joints.get(i);
                if (active(joint)) {
                    v.set(i, 0, joint.limit().lower());
                }
            }
            return v;
        }

        /**
         * qDim needs to match the actual number of moveable joints.
         * TODO: remove it.
         */
        public <Q extends Num> Vector<Q> maxQ(Nat<Q> qDim) {
            Vector<Q> v = new Vector<>(qDim);
            List<URDFModel.Joint> joints = joints();
            for (int i = 0; i < joints.size(); ++i) {
                URDFModel.Joint joint = joints.get(i);
                if (active(joint)) {
                    v.set(i, 0, joint.limit().upper());
                }
            }
            return v;
        }

        private boolean active(URDFModel.Joint joint) {
            // skip fixed since it doesn't have a parameter and thus breaks the solver
            return joint.type() == JointType.revolute
                    || joint.type() == JointType.continuous
                    || joint.type() == JointType.prismatic;
        }
    }

}
