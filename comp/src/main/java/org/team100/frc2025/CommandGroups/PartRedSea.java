package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class PartRedSea {

    public static Command get(
            Wrist2 wrist,
            Elevator elevator,
            Climber climber) {
        return sequence(
                wrist.set(0.1).until(wrist::atGoal),
                wrist.setDuty(-0.11).withTimeout(2),
                climber.setPosition(-3).until(() -> climber.atGoal()).withTimeout(4),
                parallel(
                        elevator.set(10),
                        wrist.set(0.5)));
    }
}
