package frc.robot;

import org.team100.five_bar.setups.SetupBare;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Runnable m_setup;

    public Robot() {
        /////////////////////////////
        //
        // Choose one of the setups.
        //

        // manual control of each axis
        m_setup = new SetupBare();

        // PID positional control of axes independently
        // m_setup = new SetupMech();

        // profiled control of axes independently
        // m_setup = new SetupServo();

        // cartesian coordinated control
        // m_setup = new SetupCartesian();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_setup.run();
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
