package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {

    private final SolidIndicator m_indicator;
    private final XboxController m_controller;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_controller = new XboxController(0);
        new Trigger(m_controller::getAButton)
                .whileTrue(Commands.run(() -> m_indicator.steady(Color.kGreen)))
                .onFalse(Commands.run(() -> m_indicator.steady(Color.kBlack)));
        new Trigger(m_controller::getBButton)
                .whileTrue(Commands.run(() -> m_indicator.slow(Color.kOrangeRed)))
                .onFalse(Commands.run(() -> m_indicator.steady(Color.kBlack)));
        new Trigger(m_controller::getXButton)
                .whileTrue(Commands.run(() -> m_indicator.fast(Color.kLightBlue)))
                .onFalse(Commands.run(() -> m_indicator.steady(Color.kBlack)));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_indicator.periodic();
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
