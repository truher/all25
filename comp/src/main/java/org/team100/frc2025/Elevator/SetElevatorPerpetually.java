package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorPerpetually extends Command {
    private final Elevator m_elevator;
    private final double m_value;

    public SetElevatorPerpetually(Elevator elevator, double value) {
        m_elevator = elevator;
        m_value = value;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }
}
