package org.team100.frc2025.CalgamesArm;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.trajectory.Trajectory100;

import edu.wpi.first.math.geometry.Pose2d;

/** How do the joints respond to trajectories? */
public class TrajectoryJointTest {
    /**
     * How does the smooth cartesian trajectory work in configuration space?
     * 
     * Answer: it's fine, as long as the "reach up"/"reach down" discontinuity is
     * removed.
     * 
     * Acceleration is choppy in both cartesian and configuration, so maybe turn
     * that down a bit.
     * 
     * Charts here:
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=0#gid=0
     */
    @Test
    void homeToL4() {
        TrajectoryExample e = new TrajectoryExample();
        Trajectory100 t = e.homeToL4();
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(
                0.5, 0.3);
        AnalyticalJacobian J = new AnalyticalJacobian(k);
        System.out
                .println("t, x, y, r, vx, vy, vr, ax, ay, ar, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < t.duration(); tt += 0.02) {
            SwerveControl m = SwerveControl.fromTimedPose(t.sample(tt));
            Pose2d p = m.pose();
            FieldRelativeVelocity v = m.velocity();
            FieldRelativeAcceleration a = m.acceleration();
            Config q = k.inverse(p);
            JointVelocities jv = J.inverse(m.model());
            JointAccelerations ja = J.inverseA(m);
            System.out.printf(
                    "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    tt,
                    p.getX(), p.getY(), p.getRotation().getRadians(),
                    v.x(), v.y(), v.theta(),
                    a.x(), a.y(), a.theta(),
                    q.shoulderHeight(), q.shoulderAngle(), q.wristAngle(),
                    jv.elevator(), jv.shoulder(), jv.wrist(),
                    ja.elevator(), ja.shoulder(), ja.wrist());
        }
    }

}
