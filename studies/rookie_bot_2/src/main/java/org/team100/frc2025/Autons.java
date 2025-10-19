package org.team100.frc2025;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.FeedforwardOnly;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.examples.mecanum.MecanumDrive;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Autons {
    private static final Pose2d ONE = new Pose2d(4, 1, Rotation2d.kZero);
    private static final Pose2d TWO = new Pose2d(1, 4, Rotation2d.k180deg);

    private final AutonChooser m_autonChooser;
    private final MecanumDrive m_drive;
    private final HolonomicProfile m_profile;

    public Autons(MecanumDrive drive) {
        m_autonChooser = new AutonChooser();
        m_drive = drive;
        m_profile = HolonomicProfile.wpi(4, 8, 3, 6);

        MoveAndHold one = new FeedforwardOnly<>(m_profile, ONE, m_drive);
        m_autonChooser.add("one", one.until(one::isDone).withName("auto one"));
        MoveAndHold two = new FeedforwardOnly<>(m_profile, TWO, m_drive);
        m_autonChooser.add("two", two.until(two::isDone).withName("auto two"));
        m_autonChooser.add("three",
                m_drive.driveWithGlobalVelocity(
                        new GlobalVelocityR3(1.5, 0, 0))
                        .withTimeout(1.0).withName("auto three"));
    }

    public Command get() {
        return m_autonChooser.get();
    }

}
