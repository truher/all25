package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

public class PrePlaceCoralL2 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;
    private double count = 0;
    private boolean finished = false;

    /** raise grip to scoring position perpetually */
    public PrePlaceCoralL2(Wrist2 wrist, Elevator elevator, double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
        finished = false;
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorGoal);
        if (m_elevatorGoal - 10 > m_elevator.getPosition()) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(0.55);
        }

        double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);

        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }

        if (count >= 10) {
            finished = true;
        }
    }

    public boolean isDone() {
        if (Experiments.instance.enabled(Experiment.UseProfileDone)) {
            return finished && m_wrist.profileDone() && m_elevator.profileDone();
        }
        return finished;
    }
}
