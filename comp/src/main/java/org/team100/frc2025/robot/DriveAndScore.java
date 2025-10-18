package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.ReefPoint;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a profile to drive to the specified ReefPoint, then score at the
 * specified ScoringLevel, and finally return the mechanism to home.
 */
public class DriveAndScore {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;
    private final Robot m_robot;

    /** Call only this after all the member initialization in Robot is done! */
    public DriveAndScore(Robot robot) {
        m_robot = robot;
    }

    public Command get(ScoringLevel level, ReefPoint point) {
        MoveAndHold toReef = new DriveToPoseWithProfile(
                m_robot.m_logger, m_robot.m_drive,
                m_robot.m_autoController, m_robot.m_autoProfile,
                () -> FieldConstants.makeGoal(level, point));
        MoveAndHold toL4 = m_robot.m_mech.homeToL4();
        Command eject = m_robot.m_manipulator.centerEject().withTimeout(0.5);
        return sequence(
                parallel(
                        runOnce(() -> m_robot.m_localizer.setHeedRadiusM(HEED_RADIUS_M)),
                        toReef,
                        waitUntil(toReef::isDone).andThen(toL4),
                        waitUntil(() -> toReef.isDone() && toL4.isDone())
                                .andThen(eject))
                        .until(() -> (toReef.isDone() && toL4.isDone() && eject.isFinished())),
                m_robot.m_mech.l4ToHome());
    }
}
