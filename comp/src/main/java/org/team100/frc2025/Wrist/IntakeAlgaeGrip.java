package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgaeGrip extends Command {
    private final AlgaeGrip m_grip;

    /**
     * Use high current limits to grasp the algae, and then reduce the current to
     * hold it, perpetually.
     */
    public IntakeAlgaeGrip(AlgaeGrip grip) {
        m_grip = grip;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
        m_grip.applyHighConfigs();
    }

    @Override
    public void execute() {
        m_grip.setDutyCycle(1);
        if (m_grip.hasAlgae()) {
            m_grip.applyLowConfigs();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_grip.setDutyCycle(0);

    }
}
