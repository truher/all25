package frc.robot;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private final FiveBar m_fiveBar;

    public Robot() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        XboxController controller = new XboxController(0);

        m_fiveBar = new FiveBar(logger);
        m_fiveBar.setDefaultCommand(m_fiveBar.dutyCycle(
                controller::getLeftX, controller::getRightX));

        new Trigger(controller::getAButton).whileTrue(m_fiveBar.home());
        new Trigger(controller::getBButton).onTrue(m_fiveBar.zero());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
