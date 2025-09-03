package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class CheckFunnelDanger extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private double m_initialElevatorValue;

    public CheckFunnelDanger(Wrist2 wrist, Elevator elevator) {
        m_wrist = wrist;
        m_elevator = elevator;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_initialElevatorValue = m_elevator.getPosition();
    }

    @Override
    public void execute() {
        if (m_wrist.getAngle() > 1.6) {
            m_wrist.setAngleValue(1.5);
            m_elevator.setPosition(m_initialElevatorValue);
        }
    }

    @Override
    public boolean isFinished() {
        return m_wrist.getAngle() <= 1.6;
    }
}
