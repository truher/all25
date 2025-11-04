
package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.examples.motion.OutboardRotaryPositionSubsystem;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {

    private final Viz m_viz;

    public Robot() {
        LoggerFactory log = Logging.instance().rootLogger;
        DriverXboxControl control = new DriverXboxControl(0);
        OutboardRotaryPositionSubsystem rotary = new OutboardRotaryPositionSubsystem(log);
        m_viz = new Viz(rotary::getWrappedPositionRad);
        rotary.setDefaultCommand(rotary.home());
        new Trigger(control::a).whileTrue(rotary.extend());
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_viz.run();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}
