package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

/** Raise the grip into scoring position, perpetually. */
public class PrePlaceCoralL4 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final CoralTunnel m_tunnel;
    private final double m_elevatorGoal;
    private double countElevator = 0;
    private double countWrist = 0;
    private boolean finished = false;

    /** Raise the grip into scoring position, perpetually. */
    public PrePlaceCoralL4(
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_tunnel = tunnel;
        addRequirements(m_wrist, m_elevator, m_tunnel);
    }

    @Override
    public void initialize() {
        countElevator = 0;
        countWrist = 0;
        finished = false;
    }

    @Override
    public void execute() {
        m_tunnel.setCoralMotor(1);
        m_elevator.setPosition(m_elevatorGoal);
        if (m_elevatorGoal - 10 > m_elevator.getPosition()) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(1.25);
        }

        double errorElevator = Math.abs(m_elevator.getPosition() - m_elevatorGoal);
        double errorWrist = Math.abs(m_wrist.getAngle() - 1.25);

        if (errorElevator < 0.5) {
            countElevator++;
        } else {
            countElevator = 0;
        }

        if (errorWrist < 0.05) {
            countWrist++;
        } else {
            countWrist = 0;
        }

        if (countElevator >= 2 && countWrist >= 1) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
        countElevator = 0;
        countWrist = 0;
        m_tunnel.setCoralMotor(0);
    }

    public boolean isDone() {
        if (Experiments.instance.enabled(Experiment.UseProfileDone)) {
            return finished && m_wrist.profileDone();
        } else {
            return finished;
        }
    }
}
