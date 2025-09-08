package org.team100.frc2025.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private final DrumShooter m_shooter;
    private final IndexerServo m_indexer;

    public Shoot(DrumShooter shooter, IndexerServo indexer) {
        m_shooter = shooter;
        m_indexer = indexer;
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_shooter.spinUp();
    }

    @Override
    public void execute() {
        if (m_shooter.atGoal()) {
            m_indexer.set(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_indexer.stop();
    }
}
