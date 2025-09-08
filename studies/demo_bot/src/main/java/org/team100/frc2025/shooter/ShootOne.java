package org.team100.frc2025.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootOne extends Command {
    private final DrumShooter m_shooter;
    private final IndexerSubsystem m_indexer;
    private final double distanceDeg = 90;

    private double angle;

    public ShootOne(
            DrumShooter shooter,
            IndexerSubsystem indexer) {
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
            m_indexer.set(angle + distanceDeg);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    // @Override
    // public boolean isFinished() {
    // return Math.abs(m_indexer.getAngle() - (angle + distanceDeg)) < 0.05;
    // }
}
