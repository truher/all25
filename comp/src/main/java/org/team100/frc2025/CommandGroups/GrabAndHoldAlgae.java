package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.Placeholder;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.r3.FollowTrajectory;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.wpilibj2.command.Command;

/** Intake algae and hold it forever. */
public class GrabAndHoldAlgae {
    public static Command get(
            Manipulator grip,
            Placeholder placeholder,
            Supplier<ScoringLevel> level) {
        return parallel(
                placeholder.algaeReefPick(level),
                sequence(
                        grip.algaeIntake()
                                .until(grip::hasAlgae),
                        grip.algaeHold()))
                .withName("grab and hold algae");
    }
}
