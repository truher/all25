package org.team100.frc2025.CommandGroups.ScoreSmart;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/** note unlike the L4 version this never finishes, i think due to L4 use in autos? */
public class PostDropCoralL3 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    private double initialElevatorPosition = 0;

    public PostDropCoralL3(Wrist2 wrist, Elevator elevator, double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void initialize() {
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
    }
}
