package frc.robot;

import org.team100.frc2025.CalgamesArm.CartesianSetup;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;

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
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_setup.run();
    }
}
