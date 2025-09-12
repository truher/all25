package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeAlgaeGrip extends Command {
    private final AlgaeGrip m_grip;

    public OuttakeAlgaeGrip(AlgaeGrip grip) {
        m_grip = grip;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
        m_grip.applyHighConfigs();
    }

    @Override
    public void execute() {
        m_grip.setDutyCycle(-0.1);
    }

    public void end(boolean interrupted) {
        m_grip.stop();
    }
}
