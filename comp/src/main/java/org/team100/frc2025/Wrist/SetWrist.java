package org.team100.frc2025.Wrist;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWrist extends Command {
    private final Wrist2 m_wrist;
    private final double m_angle;
    private double count = 0;

    public SetWrist(Wrist2 wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        m_wrist.setAngleValue(m_angle);
        if (Math.abs(m_wrist.getAngle() - m_angle) < 0.05) {
            count++;
        } else {
            count = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setWristDutyCycle(0);
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (count < 5)
            return false;
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return m_wrist.profileDone();
        return true;
    }
}
