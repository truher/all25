package frc.robot;

import org.team100.lib.motion.lynxmotion_arm.LynxArmConfig;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics;
import org.team100.lib.motion.lynxmotion_arm.LynxArmPose;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lynxmotion AL5D trainer board.
 * 
 * The arm is a serial mechanism, and so it implements a series of
 * three-dimensional transforms: rotations (the joints) and translations (along
 * the links).
 * 
 * Since the AL5D is mostly planar, we're using the 2d convention, where each
 * link is a translation along its own x, and each joint rotation is also
 * aligned with the same parent link x.
 * 
 * This means that the "all joints at zero" state for the arm should be
 * stretched out along the world x axis.
 * 
 * overview:
 * 
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 *
 * calibration:
 * 
 * https://docs.google.com/spreadsheets/d/1XCtQGnJABVWTuCkx0t6u_xJjezelqyBE2Rz7eqmoyh4
 * 
 * background:
 * 
 * https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html
 * 
 * TODO: calibrate the PWM pulse width range.
 * TODO: move this to lib when it's done.
 * TODO: compute gravity effect on each joint, adjust estimated position
 */
public class LynxArm extends SubsystemBase implements AutoCloseable {
    private static final boolean DEBUG = false;
    private static final Pose3d HOME = new Pose3d(0.2, 0, 0.2, new Rotation3d(0, Math.PI / 4, 0));

    private final CalibratedServo m_swing;
    private final CalibratedServo m_boom;
    private final CalibratedServo m_stick;
    private final CalibratedServo m_wrist;
    private final CalibratedServo m_twist;
    private final CalibratedServo m_grip;

    private final LynxArmKinematics m_kinematics;

    public LynxArm(LynxArmKinematics kinematics) {
        m_kinematics = kinematics;
        // all these implement the WPI normal coordinates:
        // x ahead, y left, z up.

        // yaw; joint zero is is in the middle of the servo range; unconstrained.
        m_swing = new CalibratedServo(0,
                new Clamp(-Math.PI / 2, Math.PI / 2),
                new AffineFunction(-3.216, 1.534));

        // pitch; joint zero and servo zero are aligned; unconstrained.
        m_boom = new CalibratedServo(1,
                new Clamp(-Math.PI, Math.PI),
                new AffineFunction(-Math.PI, 0));

        // pitch; joint zero is servo max; constrained in the positive direction.
        m_stick = new CalibratedServo(2,
                new Clamp(0, Math.PI),
                new AffineFunction(-Math.PI, Math.PI));

        // pitch; joint zero is in the middle of the servo range; unconstrained
        m_wrist = new CalibratedServo(3,
                new Clamp(-Math.PI / 2, Math.PI / 2),
                new AffineFunction(-Math.PI, Math.PI / 2));

        // roll; joint zero is in the middle of the servo range; unconstrained.
        m_twist = new CalibratedServo(4,
                new Clamp(-Math.PI / 2, Math.PI / 2),
                new AffineFunction(-Math.PI, Math.PI / 2));

        // the grip axis measures the width of the jaws.
        // TODO: calibrate in meters
        m_grip = new CalibratedServo(5,
                new Clamp(0, 0.02),
                new AffineFunction(-0.02, 0.02));

        // initialize the servo values so that the solver doesn't freak out
        // TODO: why was it freaking out?
        m_swing.setAngle(0);
        m_boom.setAngle(-2.0 * Math.PI / 3);
        m_stick.setAngle(Math.PI / 2);
        m_wrist.setAngle(Math.PI / 2);
        m_twist.setAngle(0);
        m_grip.setAngle(0);

        // set the initial position to the actual desired value
        if (DEBUG) {
            System.out.println("-> initializing position to HOME");
            System.out.printf("HOME: %s\n", Util.poseStr(HOME));
        }
        setPosition(HOME);
        if (DEBUG)
            System.out.println("-> check position");
        getPosition();
        if (DEBUG)
            System.out.println("-> initializing done");
    }

    /**
     * Sets the end-effector pose.
     * 
     * Moves in an uncoordinated way, so the new position should be close to the
     * current position. Coordination should be handled by the caller.
     * 
     * For indeterminate axes, we do nothing.
     * 
     * TODO: use the previous value for indeterminate axes.
     */
    public void setPosition(Pose3d end) {
        if (DEBUG)
            System.out.println("setPosition()");
        LynxArmConfig q = getInverse(end);
        if (DEBUG)
            System.out.printf("set q: %s\n", q.str());

        q.swing().ifPresent(m_swing::setAngle);
        m_boom.setAngle(q.boom());
        m_stick.setAngle(q.stick());
        m_wrist.setAngle(q.wrist());
        q.twist().ifPresent(m_twist::setAngle);
        if (q.swing().isEmpty())
            if (DEBUG)
                System.out.println("empty swing");
        if (q.twist().isEmpty())
            if (DEBUG)
                System.out.println("empty twist");
    }

    public LynxArmPose getPosition() {
        if (DEBUG)
            System.out.println("getPosition()");
        LynxArmConfig q = getMeasuredConfig();
        LynxArmPose p = m_kinematics.forward(q);
        if (DEBUG)
            System.out.printf("p6: %s\n", Util.poseStr(p.p6()));
        return p;
    }

    /** Move to goal forever -- use a termination condition to stop. */
    public MoveCommand moveTo(Pose3d goal) {
        return new MoveCommand(this, goal, 0.1);
    }

    /** Move to goal and terminate when done. */
    public Command moveQuicklyUntilDone(Pose3d goal) {
        MoveCommand m = new MoveCommand(this, goal, 0.5);
        return m.until(m::done);
    }

    public MoveCommand moveHome() {
        return new MoveCommand(this, HOME, 0.1);
    }

    @Override
    public void close() {
        m_swing.close();
        m_boom.close();
        m_stick.close();
        m_wrist.close();
        m_twist.close();
        m_grip.close();
    }

    /////////////////////////////////////////////

    LynxArmConfig getInverse(Pose3d p) {
        if (DEBUG)
            System.out.println("getInverse()");
        LynxArmConfig q0 = getMeasuredConfig();
        // p might be infeasible, so fix it.
        p = LynxArmKinematics.fix(p);
        return m_kinematics.inverse(q0, p);
    }

    LynxArmConfig getMeasuredConfig() {
        if (DEBUG)
            System.out.println("getMeasuredConfig()");
        LynxArmConfig q = new LynxArmConfig(
                m_swing.getAngle(),
                m_boom.getAngle(),
                m_stick.getAngle(),
                m_wrist.getAngle(),
                m_twist.getAngle());
        if (DEBUG)
            System.out.printf("get q: %s\n", q.str());
        return q;
    }
}
