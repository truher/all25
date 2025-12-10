package org.team100.lib.subsystems.r3.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.targeting.Targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a sequence of commands to drive nearby a target with the intake facing
 * it, and then sweep the intake over the target.
 */
public class FloorPickSequence {
    /** Landing point distance to target */
    private static final double LANDING_DISTANCE = 1.0;
    /** End effector relative to robot */
    private static final Transform2d END_EFFECTOR = new Transform2d(-0.5, 0.193, Rotation2d.k180deg);

    public static Command get(
            LoggerFactory parent,
            LoggerFactory fieldLog,
            VelocitySubsystemR3 drive,
            Targets targets,
            ControllerR3 controller,
            HolonomicProfile profile) {
        LoggerFactory log = parent.type(FloorPickSequence.class);
        // target translation is not offset at all -- command does the offsetting.
        Supplier<Optional<Translation2d>> target = targets::getClosestTarget;
        // runway translation
        Supplier<Optional<Translation2d>> runway = () -> {
            Optional<Translation2d> t = target.get();
            if (t.isEmpty())
                return t;
            Translation2d goal = t.get();
            Translation2d robot = drive.getState().pose().getTranslation();
            Rotation2d absoluteCourse = goal.minus(robot).getAngle();
            // landing point is along the line from the current robot location to the
            // target.
            // this makes for odd paths when the robot is moving
            // TODO: make a polar ring-target controller to fix that
            Translation2d offset = new Translation2d(LANDING_DISTANCE, absoluteCourse);
            // back up to find the landing point
            Translation2d landing = goal.minus(offset);
            return Optional.of(landing);
        };

        DriveToTranslationWithOffsetUsingProfile toRunway = new DriveToTranslationWithOffsetUsingProfile(
                log, fieldLog, runway, drive, controller, profile, END_EFFECTOR);

        DriveToTranslationWithOffsetUsingProfile toTarget = new DriveToTranslationWithOffsetUsingProfile(
                log, fieldLog, target, drive, controller, profile, END_EFFECTOR);

        return sequence(
                toRunway.until(toRunway::thetaAligned),
                toTarget.until(toTarget::isDone));
    }

}
