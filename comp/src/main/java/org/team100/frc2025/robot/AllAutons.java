package org.team100.frc2025.robot;

import org.team100.lib.config.AutonChooser;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 */
public class AllAutons {
    private final AutonChooser m_autonChooser;

    /** Call only this after all the member initialization in Robot is done! */
    public AllAutons(Robot robot) {
        m_autonChooser = new AutonChooser();
        HolonomicProfile m_autoProfile;
        FullStateSwerveController m_autoController;
        m_autoProfile = HolonomicProfile.currentLimitedExponential(1, 2, 4,
                robot.m_swerveKinodynamics.getMaxAngleSpeedRad_S(), robot.m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                5);
        m_autoController = SwerveControllerFactory.auto2025LooseTolerance(robot.m_logger.name("Auton"));

        // WARNING! The glass widget will override the default, so check it!
        // Run the auto in pre-match testing!
        m_autonChooser.addAsDefault("Lollipop", new LolipopAuto(robot, m_autoProfile, m_autoController).get());

        DriveAndScore driveAndScore = new DriveAndScore(robot, m_autoProfile, m_autoController);
        m_autonChooser.add("Coral 1 left", driveAndScore.get(ScoringLevel.L4, ReefPoint.J));
        m_autonChooser.add("Coral 1 mid", driveAndScore.get(ScoringLevel.L4, ReefPoint.H));
        m_autonChooser.add("Coral 1 right", driveAndScore.get(ScoringLevel.L4, ReefPoint.F));

        Auton auton = new Auton(robot, m_autoProfile, m_autoController);
        m_autonChooser.add("Left Preload Only", auton.leftPreloadOnly());
        m_autonChooser.add("Center Preload Only", auton.centerPreloadOnly());
        m_autonChooser.add("Right Preload Only", auton.rightPreloadOnly());
        m_autonChooser.add("Left Three Coral", auton.left());
        m_autonChooser.add("Right Three Coral", auton.right());
    }

    public Command get() {
        return m_autonChooser.get();
    }

}
