package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevator extends Command {
    private final Elevator m_elevator;
    private final double m_goal;

    /** set position via profile until done. */
    public SetElevator(Elevator elevator, double goal) {
        m_elevator = elevator;
        m_goal = goal;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_goal);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atGoal();
    }
}
