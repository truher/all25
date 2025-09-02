package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristPerpetually extends Command {
    private final Wrist2 m_wrist;
    private final double m_angle;

    public SetWristPerpetually(Wrist2 wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle;
        addRequirements(m_wrist);
    }

    @Override
    public void execute() {
        m_wrist.setAngleValue(m_angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setWristDutyCycle(0);
    }

}
