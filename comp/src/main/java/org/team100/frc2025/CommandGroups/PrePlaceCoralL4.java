package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/** Raise the grip into scoring position, perpetually. */
public class PrePlaceCoralL4 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final CoralTunnel m_tunnel;
    private final double m_elevatorGoal;

    /** Raise the grip into scoring position, perpetually. */
    public PrePlaceCoralL4(
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_tunnel = tunnel;
        addRequirements(m_wrist, m_elevator, m_tunnel);
    }



    @Override
    public void execute() {
        m_tunnel.setCoralMotor(1);
        m_elevator.setPosition(m_elevatorGoal);
        if (isHigh()) {
            // out when high enough
            m_wrist.setAngleValue(1.25);
        } else {
            // in on the way up
            m_wrist.setAngleValue(0.4);
        }
    }

    private boolean isHigh() {
        return m_elevator.getPosition() > m_elevatorGoal - 10;
    }

    @Override
    public void end(boolean interrupted) {
        m_tunnel.setCoralMotor(0);
    }

    /** Both elevator and wrist are at their goals. */
    public boolean isDone() {
        // double check isHigh to make sure we're using the right wrist goal
        return isHigh() && m_elevator.atGoal() && m_wrist.atGoal();
    }
}
