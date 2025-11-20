package org.team100.frc2025;

import org.team100.lib.subsystems.shooter.IndexerServo;
import org.team100.lib.subsystems.shooter.SingleDrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Illustrates a command with a condition. This can also be done using the
 * "fluent" method shown in RobotContainer.
 */
public class SingleShoot extends Command {
    private final SingleDrumShooter m_shooter;
    private final IndexerServo m_indexer;

    public SingleShoot(SingleDrumShooter shooter, IndexerServo indexer) {
        m_shooter = shooter;
        m_indexer = indexer;
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_shooter.spinUp(-7);
    }

    @Override
    public void execute() {
        if (m_shooter.atGoal()) {
            m_indexer.setPosition(0.7);
        }
    }
}
