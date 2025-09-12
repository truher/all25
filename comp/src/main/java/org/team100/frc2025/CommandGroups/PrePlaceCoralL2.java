package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class PrePlaceCoralL2 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    /** raise grip to scoring position perpetually */
    public PrePlaceCoralL2(Wrist2 wrist, Elevator elevator, double elevatorGoal) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorGoal;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorGoal);
        if (isHigh()) {
            m_wrist.setAngleValue(0.55);
        } else {
            m_wrist.setAngleValue(0.4);
        }
    }

    private boolean isHigh() {
        return m_elevator.getPosition() > m_elevatorGoal - 10;
    }

    public boolean isDone() {
        return isHigh() && m_elevator.atGoal() && m_wrist.atGoal();
    }
}
