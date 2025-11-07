package org.team100.frc2025;

import org.team100.lib.subsystems.shooter.DualDrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Illustrates a command with a condition. This can also be done using the
 * "fluent" method shown in RobotContainer.
 */
public class Shoot extends Command {
    private final DualDrumShooter m_shooter;
    private final IndexerServo m_indexer;
    private final double m_speed;

    public Shoot(DualDrumShooter shooter, IndexerServo indexer, double speed) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_speed = speed;
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_shooter.spinUp(m_speed);
    }

    @Override
    public void execute() {
        if (m_shooter.atGoal()) {
            m_indexer.set(1);
        }
    }
}