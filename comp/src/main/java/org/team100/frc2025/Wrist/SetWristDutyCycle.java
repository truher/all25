package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristDutyCycle extends Command {
    private final Wrist2 m_wrist;
    private final double m_duty;

    public SetWristDutyCycle(Wrist2 wrist, double duty) {
        m_wrist = wrist;
        m_duty = duty;
        addRequirements(m_wrist);
    }

    @Override
    public void execute() {
        m_wrist.setWristDutyCycle(m_duty);
    }

}
