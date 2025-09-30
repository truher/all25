package org.team100.frc2025.CalgamesArm;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.trajectory.Trajectory100;

/** How do the joints respond to trajectories? */
public class TrajectoryJointTest {
    @Test
    void test0() {
        TrajectoryExample e = new TrajectoryExample();
        Trajectory100 t = e.pickToL4();
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(
                0.5, 0.3);
        AnalyticalJacobian J = new AnalyticalJacobian(k);
        System.out.println("t, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < t.duration(); tt += 0.02) {
            SwerveControl m = SwerveControl.fromTimedPose(t.sample(tt));
            Config q = k.inverse(m.pose());
            JointVelocities jv = J.inverse(m.model());
            JointAccelerations ja = J.inverseA(m);
            System.out.printf(
                    "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    tt,
                    q.shoulderHeight(), q.shoulderAngle(), q.wristAngle(),
                    jv.elevator(), jv.shoulder(), jv.wrist(),
                    ja.elevator(), ja.shoulder(), ja.wrist());
        }
    }

}
