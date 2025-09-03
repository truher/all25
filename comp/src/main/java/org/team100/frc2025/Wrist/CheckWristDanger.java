package org.team100.frc2025.Wrist;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

/** Move the wrist out of the way of the elevator. */
public class CheckWristDanger extends Command {
    private static final double SAFE = 0.1;

    private final Wrist2 m_wrist;

    public CheckWristDanger(Wrist2 wrist) {
        m_wrist = wrist;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_wrist.getAngle() < SAFE) {
            // a little extra to push it beyond the safe boundary
            m_wrist.setAngleValue(SAFE + 0.05);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setWristDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        // wrist is out of the way, maybe far out
        return m_wrist.getAngle() > SAFE;
    }
}
