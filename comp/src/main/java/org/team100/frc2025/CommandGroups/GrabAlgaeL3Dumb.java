package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckWristDanger;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class GrabAlgaeL3Dumb {

    public static Command get(
            Wrist2 wrist,
            Elevator elevator,
            AlgaeGrip grip) {
        return sequence(
                new CheckWristDanger(wrist),
                parallel(
                        elevator.set(35),
                        wrist.set(3.7),
                        new IntakeAlgaeGrip(grip)));
    }
}
