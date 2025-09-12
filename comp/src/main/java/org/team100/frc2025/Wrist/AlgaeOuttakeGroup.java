package org.team100.frc2025.Wrist;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeOuttakeGroup {
    public static Command get(
            AlgaeGrip grip,
            Wrist2 wrist,
            Elevator elevator) {
        return parallel(
                new OuttakeAlgaeGrip(grip),
                wrist.set(3.7),
                elevator.set(12));
    }
}
