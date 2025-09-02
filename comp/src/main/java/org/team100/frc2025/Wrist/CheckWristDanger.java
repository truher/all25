package org.team100.frc2025.Wrist;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

/** Move the wrist out of the way of the elevator. */
public class CheckWristDanger extends Command {
    private static final double SAFE = 0.1;

    private final Wrist2 m_wrist;
    private boolean finished = false;
    private double count = 0;

    public CheckWristDanger(Wrist2 wrist) {
        m_wrist = wrist;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        finished = false;
        count = 0;
    }

    @Override
    public void execute() {
        if (m_wrist.getAngle() < SAFE) {
            m_wrist.setAngleValue(SAFE);
            if (Math.abs(m_wrist.getAngle() - SAFE) < 0.05) {
                count++;
            } else {
                count = 0;
            }
        } else {
            finished = true;
        }
        if (count >= 20) {
            // if we've been trying for awhile (about 0.5 sec), give up.
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setWristDutyCycle(0);
        finished = false;
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return finished && m_wrist.profileDone();
        return finished;
    }
}
