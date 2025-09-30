package org.team100.frc2025.CalgamesArm;

import java.util.function.Supplier;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.kinematics.JointVelocities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Use the operator control to "fly" the arm around in config space. */
public class ManualCartesian extends Command {
    private static final boolean DEBUG = false;

    private final Supplier<DriverControl.Velocity> m_input;
    private final CalgamesMech m_subsystem;

    private Pose2d m_pose;
    private JointVelocities m_prev;

    public ManualCartesian(
            Supplier<DriverControl.Velocity> input,
            CalgamesMech subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_pose = m_subsystem.getState().pose();
        m_prev = new JointVelocities(0, 0, 0);
    }

    @Override
    public void execute() {

        // input is [-1, 1]
        DriverControl.Velocity input = m_input.get();
        final double dt = 0.02;
        // control is velocity.
        // velocity in m/s and rad/s
        // we want full scale to be about 0.5 m/s and 0.5 rad/s
        FieldRelativeVelocity jv = new FieldRelativeVelocity(
                input.x() * 1.5,
                input.y() * 1.5,
                input.theta() * 3);

        double x2 = m_pose.getX() + jv.x() * dt;
        double y2 = m_pose.getY() + jv.y() * dt;
        Rotation2d r2 = m_pose.getRotation().plus(new Rotation2d(jv.theta() * dt));
        m_pose = new Pose2d(x2, y2, r2); // our new goal point

        m_subsystem.set(new SwerveControl(m_pose));
        if (DEBUG)
            System.out.printf("pose %s\n", m_pose);

        // // impose limits; see CalgamesMech for more limits.
        // if (newC.shoulderHeight() < 0 || newC.shoulderHeight() > 1.7) {
        // newC = new Config(m_config.shoulderHeight(), newC.shoulderAngle(),
        // newC.wristAngle());
        // }
        // if (newC.shoulderAngle() < -2 || newC.shoulderAngle() > 2) {
        // newC = new Config(newC.shoulderHeight(), m_config.shoulderAngle(),
        // newC.wristAngle());
        // }
        // if (newC.wristAngle() < -1.5 || newC.wristAngle() > 2.1) {
        // newC = new Config(newC.shoulderHeight(), newC.shoulderAngle(),
        // m_config.wristAngle());
        // }

        // recompute velocity and accel
        // JointVelocities newJv = newC.diff(m_config, dt);
        // JointAccelerations ja = newJv.diff(m_prev, dt);

        // m_subsystem.set(newC, newJv, ja);
        // m_subsystem.set(newC, newJv, ja);
        // m_config = newC;
        // m_prev = newJv;
    }
}
