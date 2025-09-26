package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == reef lock
 * left bumper button     == slow speed
 * left stick x [-1,1]    == omega
 * left stick y [-1,1]    == **** unbound ****
 * left stick button      == (don't use)
 * dpad/pov angle [0,360] == snaps
 * "back" button          == reset 0 rotation
 * "start" button         == reset 180 rotation
 * right stick x [-1,1]   == x velocity
 * right stick y [-1,1]   == y velocity
 * right stick button     == (don't use)
 * x button               ==
 * y button               ==
 * a button               == drive to score at reef
 * b button               == **** unbound ****
 * right trigger [0,1]    == medium speed
 * right bumper button    == barge assist
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl implements DriverControl {
    private static final double DEADBAND = 0.1;
    private static final double EXPO = 0.65;
    private static final double MEDIUM = 0.5;
    private static final double SLOW = 0.15;

    private final XboxController m_controller;
    private final DoubleLogger m_log_right_y;
    private final DoubleLogger m_log_right_x;
    private final DoubleLogger m_log_left_x;
    private final EnumLogger m_log_speed;

    public DriverXboxControl(LoggerFactory parent) {
        m_controller = new XboxController(0);
        LoggerFactory child = parent.type(this);
        m_log_right_y = child.doubleLogger(Level.TRACE, "Xbox/right y");
        m_log_right_x = child.doubleLogger(Level.TRACE, "Xbox/right x");
        m_log_left_x = child.doubleLogger(Level.TRACE, "Xbox/left x");
        m_log_speed = child.enumLogger(Level.TRACE, "control_speed");
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    /**
     * Applies expo to the magnitude of the cartesian input, since these are "round"
     * joysticks.
     */
    @Override
    public Velocity velocity() {
        final double rightY = m_controller.getRightY();
        final double rightX = m_controller.getRightX();
        final double leftX = m_controller.getLeftX();
        m_log_right_y.log(() -> rightY);
        m_log_right_x.log(() -> rightX);
        m_log_left_x.log(() -> leftX);

        double dx = 0;
        double dy = 0;
        double x = -1.0 * clamp(rightY, 1);
        double y = -1.0 * clamp(rightX, 1);
        double r = Math.hypot(x, y);
        if (r > DEADBAND) {
            double expoR = expo(r, EXPO);
            double ratio = expoR / r;
            dx = ratio * x;
            dy = ratio * y;
        } else {
            dx = 0;
            dy = 0;
        }

        double dtheta = expo(deadband(-1.0 * clamp(leftX, 1), DEADBAND, 1), EXPO);

        Speed speed = speed();
        m_log_speed.log(() -> speed);

        switch (speed) {
            case SLOW:
                return new Velocity(SLOW * dx, SLOW * dy, SLOW * dtheta);
            case MEDIUM:
                return new Velocity(MEDIUM * dx, MEDIUM * dy, MEDIUM * dtheta);
            default:
                return new Velocity(dx, dy, dtheta);
        }
    }

    private Speed speed() {
        if (m_controller.getLeftBumperButton())
            return Speed.SLOW;
        if (m_controller.getRightTriggerAxis() > .9)
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    @Override
    public boolean resetRotation0() {
        return m_controller.getBackButton();
    }

    @Override
    public boolean resetRotation180() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean toReef() {
        return m_controller.getAButton();
    }

    @Override
    public boolean feedFunnel() {
        return m_controller.getXButton();
    }

    @Override
    public boolean test() {
        return false;
    }

    @Override
    public boolean useReefLock() {
        if (m_controller.getLeftTriggerAxis() > 0.9) {
            return true;
        }
        return false;
    }

    @Override
    public boolean driveWithBargeAssist() {
        return m_controller.getRightBumperButton();
    }

    @Override
    public boolean climb() {
        return m_controller.getYButton();
    }

    // These are for prototyping with Xbox controllers.
    // Please don't use these for comp.
    @Override
    public boolean x() {
        return m_controller.getXButton();
    }

    @Override
    public boolean y() {
        return m_controller.getYButton();
    }

    @Override
    public boolean a() {
        return m_controller.getAButton();
    }

    @Override
    public boolean b() {
        return m_controller.getBButton();
    }

}
