package org.team100.frc2025;

import org.team100.lib.subsystems.shooter.SingleDrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Illustrates a command with a condition. This can also be done using the
 * "fluent" method shown in RobotContainer.
 */
public class SingleIntake extends Command {
    private final SingleDrumShooter m_shooter;

    public SingleIntake(SingleDrumShooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.spinUp(5);
    }
}
