package frc.robot;

import java.util.function.DoubleSupplier;

import org.team100.lib.framework.TimedRobot100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move in the XY plane, leave z alone, leave the grip alone.
 * The end-effector pose is automatic:
 * * Roll is always zero.
 * * Pitch depends on range from the origin
 * * Yaw follows the XY position.
 */
public class MoveManually extends Command {
    private static final boolean DEBUG = false;

    // prefer vertical grip.  beyond this radius, extend it.
    private static final double VERTICAL_LIMIT = 0.3;
    // TODO: find the actual limits
    private static final double MIN_RADIUS = 0.1;
    private static final double MAX_RADIUS = 0.4;
    // meters per second
    private static final double SPEED = 0.1;
    // step per loop
    private static final double STEP = TimedRobot100.LOOP_PERIOD_S * SPEED;

    private final LynxArm m_arm;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private double m_grip;

    private double m_x;
    private double m_y;
    private double m_z;

    public MoveManually(LynxArm arm, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        m_arm = arm;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_grip = m_arm.getGrip();
        Pose3d start = m_arm.getPosition().p6();
        m_x = start.getX();
        m_y = start.getY();
        m_z = start.getZ();
    }

    @Override
    public void execute() {
        if (DEBUG)
            System.out.println("\n***EXECUTE***");

        double r = Math.hypot(m_x, m_y);
        double yaw = Math.atan2(m_y, m_x);
        double r1 = Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);

        if (r1 <= MIN_RADIUS + STEP) {
            // move away from the origin
            m_x += STEP * Math.cos(yaw);
            m_y += STEP * Math.sin(yaw);
        } else if (r1 >= MAX_RADIUS - STEP) {
            // move away from the edge
            m_x -= STEP * Math.cos(yaw);
            m_y -= STEP * Math.sin(yaw);
        } else if (m_x <= STEP) {
            // move away from the x axis
            m_x += STEP;
        } else {
            // driver update is ok
            m_x = m_x + m_xSpeed.getAsDouble() * STEP;
            m_y = m_y + m_ySpeed.getAsDouble() * STEP;

        }

        if (DEBUG) {
            System.out.printf("x %f y %f\n", m_x, m_y);
        }

        double roll = 0.0;
        final double pitch;
        if (r < VERTICAL_LIMIT) {
            // for near targets, the grip is vertical.
            pitch = Math.PI / 2;
        } else {
            // further away, the pitch depends on range.
            double s = 1 - (r - VERTICAL_LIMIT) / (MAX_RADIUS - VERTICAL_LIMIT);
            pitch = s * Math.PI / 2;
        }
        Pose3d newPose = new Pose3d(
                new Translation3d(m_x, m_y, m_z),
                new Rotation3d(roll, pitch, yaw));

        m_arm.setGrip(m_grip);
        m_arm.setPosition(newPose);
    }

}
