package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.wpilibj2.command.Command;

/** Intake algae and hold it forever. */
public class GrabAndHoldAlgae {

    public static double fromLevel(Supplier<ScoringLevel> level) {
        return switch (level.get()) {
            case L2 -> 23.0;
            case L3 -> 35.0;
            default -> throw new IllegalArgumentException(
                    "Invalid algae level should never happen");
        };
    }

    public static Command get(
            Wrist2 wrist,
            Elevator elevator,
            Manipulator grip,
            Supplier<ScoringLevel> level) {

        return parallel(
                elevator.set(() -> fromLevel(level)),
                wrist.set(3.7),
                sequence(
                        grip.algaeIntake()
                                .until(grip::hasAlgae),
                        grip.algaeHold()))
                .withName("grab and hold algae");
    }

}
