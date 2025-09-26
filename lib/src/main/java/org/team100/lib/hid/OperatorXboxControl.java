package org.team100.lib.hid;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == 
 * left bumper button     ==
 * left stick x [-1,1]    == manual climb speed
 * left stick y [-1,1]    == 
 * left stick button      == (don't use)
 * dpad/pov angle [0,360] == 
 * "back" button          == 
 * "start" button         == 
 * right stick x [-1,1]   == 
 * right stick y [-1,1]   == 
 * right stick button     == (don't use)
 * x button               == extend and spin climber for intake
 * y button               == activate manual climb mode
 * a button               == 
 * b button               == 
 * right trigger [0,1]    ==
 * right bumper button    ==
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class OperatorXboxControl implements OperatorControl {
    private final XboxController m_controller;

    public OperatorXboxControl() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public double manualClimbSpeed() {
        return m_controller.getLeftX();
    }

    @Override
    public boolean activateManualClimb() {
        return m_controller.getYButton();
    }

    @Override
    public boolean climbIntake() {
        return m_controller.getXButton();
    }

    @Override
    public boolean intake() {
        return m_controller.getAButton();
    }

    @Override
    public boolean outtake() {
        return m_controller.getBButton();
    }

}
