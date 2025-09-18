package frc.robot;

import org.team100.CartesianSetup;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Runnable m_setup;

    public Robot() {
        // m_setup = new Setup();
        m_setup = new CartesianSetup();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_setup.run();
    }
}
