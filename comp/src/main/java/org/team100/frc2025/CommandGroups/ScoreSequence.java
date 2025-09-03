package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/** Drop the grip on the peg and then stow. */
public class ScoreSequence {
    public static Command get(
            Wrist2 wrist,
            Elevator elevator,
            double elevatorMid, // pull the wrist in here
            double elevatorGoal) { // stow
        return sequence(
                parallel(
                        elevator.set(elevatorGoal),
                        wrist.set(
                                () -> elevator.getPosition() > elevatorMid ? 1.25 : 0.4))
                        .until(elevator::atGoal),
                parallel(
                        elevator.set(0.1),
                        wrist.set(0.1))
                        .until(() -> elevator.atGoal() && wrist.atGoal()),
                wrist.setDuty(-0.15));
    }

}
