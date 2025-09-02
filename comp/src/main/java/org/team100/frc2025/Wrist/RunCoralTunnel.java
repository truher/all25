package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class RunCoralTunnel extends Command {
    private final CoralTunnel m_tunnel;
    private final double m_value;

    /** run the tunnel perpetually */
    public RunCoralTunnel(CoralTunnel tunnel, double value) {
        m_tunnel = tunnel;
        m_value = value;
        addRequirements(m_tunnel);
    }

    @Override
    public void execute() {
        m_tunnel.setCoralMotor(m_value);
    }

    @Override
    public void end(boolean interrupted) {
        m_tunnel.setCoralMotor(0);
    }

}
