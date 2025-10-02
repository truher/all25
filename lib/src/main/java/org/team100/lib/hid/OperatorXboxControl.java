package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.hid.DriverControl.Velocity;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == 
 * left bumper button     == activate manual arm
 * left stick x [-1,1]    == manual wrist rotation
 * left stick y [-1,1]    == manual climb speed
 * left stick button      == (don't use)
 * dpad/pov angle [0,360] == 
 * "back" button          == 
 * "start" button         == 
 * right stick x [-1,1]   == manual shoulder rotation
 * right stick y [-1,1]   == manual elevator
 * right stick button     == (don't use)
 * x button               == COMMAND ISN'T MADE YET ()
 * y button               == manual intake coral (mapped to "intake")
 * a button               == COMMAND ISN'T MADE YET
 * b button               == manual puke coral (mapped to "outake")
 * right trigger [0,1]    == 
 * right bumper button    == activate manual climb
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class OperatorXboxControl implements OperatorControl {
    private static final double DEADBAND = 0.1;
    private static final double EXPO = 0.65;

    private final XboxController m_controller;

    public OperatorXboxControl() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    /**
     * dx = right y = axis 5
     * dy = right x = axis 4
     * dr = left x = axis 0
     */
    @Override
    public Velocity velocity() {
        final double rightY = m_controller.getRightY();
        final double rightX = m_controller.getRightX();
        final double leftX = m_controller.getLeftX();
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

        return new Velocity(dx, dy, dtheta);
    }

    @Override
    public boolean manual() {
        return m_controller.getLeftBumperButton();
    }

    @Override
    public double manualClimbSpeed() {
        return m_controller.getLeftY();
    }

    @Override
    public boolean activateManualClimb() {
        return m_controller.getRightBumperButton();
    }

    @Override
    public boolean climbIntake() {
        if (m_controller.getRightTriggerAxis() > .9) {
            return true;
        }
        return false;
    }

    // TODO: Make work
    @Override
    public boolean intake() {
        return m_controller.getAButton();
    }

    // TODO: Make work
    @Override
    public boolean outtake() {
        return m_controller.getBButton();
    }

}
