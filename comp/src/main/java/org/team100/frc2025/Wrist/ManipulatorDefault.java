package org.team100.frc2025.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

public class ManipulatorDefault extends Command {
    private final Manipulator m_manipulator;

    public ManipulatorDefault(Manipulator manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        m_manipulator.stop();
    }
}