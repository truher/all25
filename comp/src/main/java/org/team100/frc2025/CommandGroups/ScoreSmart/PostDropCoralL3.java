package org.team100.frc2025.CommandGroups.ScoreSmart;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class PostDropCoralL3 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    private double count = 0;
    private double initialElevatorPosition = 0;

    public PostDropCoralL3(Wrist2 wrist, Elevator elevator, double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
        initialElevatorPosition = m_elevator.getPosition();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorGoal);

        if (Math.abs(m_elevator.getPosition() - initialElevatorPosition) > 0.5) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(0.9);
        }

        double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);

        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }

        if (count >= 5) {
            // finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
