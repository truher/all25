package org.team100.frc2025.Elevator;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevator extends Command {
    private final Elevator m_elevator;
    private final double m_value;
    private final IntLogger m_log_count;
    private final DoubleLogger m_log_error;

    private int count = 0;

    public SetElevator(
            LoggerFactory logger,
            Elevator elevator,
            double value) {
        LoggerFactory child = logger.type(this);
        m_log_count = child.intLogger(Level.TRACE, "count");
        m_log_error = child.doubleLogger(Level.TRACE, "error");
        m_elevator = elevator;
        m_value = value;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3

        double error = Math.abs(m_elevator.getPosition() - m_value);
        m_log_error.log(() -> error);
        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }
        m_log_count.log(() -> count);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (count < 5)
            return false;
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return m_elevator.profileDone();
        return true;
    }
}
