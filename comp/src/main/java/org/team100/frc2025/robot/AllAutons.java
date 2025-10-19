package org.team100.frc2025.robot;

import org.team100.lib.config.AutonChooser;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 */
public class AllAutons {
    private final AutonChooser m_autonChooser;

    public AllAutons(Machinery machinery) {
        m_autonChooser = new AutonChooser();
        final LoggerFactory logger = Logging.instance().rootLogger;

        final HolonomicProfile profile = HolonomicProfile.currentLimitedExponential(1, 2, 4,
                machinery.m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                machinery.m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                5);
        final FullStateSwerveController controller = SwerveControllerFactory
                .auto2025LooseTolerance(logger.name("Auton"));
        final TrajectoryPlanner planner = new TrajectoryPlanner(
                new TimingConstraintFactory(machinery.m_swerveKinodynamics).medium());

        // WARNING! The glass widget will override the default, so check it!
        // Run the auto in pre-match testing!
        m_autonChooser.addAsDefault("Lollipop",
                new LolipopAuto(logger, machinery, profile, controller, planner).get());

        DriveAndScore driveAndScore = new DriveAndScore(logger, machinery, profile, controller);
        m_autonChooser.add("Coral 1 left", driveAndScore.get(ScoringLevel.L4, ReefPoint.J));
        m_autonChooser.add("Coral 1 mid", driveAndScore.get(ScoringLevel.L4, ReefPoint.H));
        m_autonChooser.add("Coral 1 right", driveAndScore.get(ScoringLevel.L4, ReefPoint.F));

        Auton auton = new Auton(logger, machinery, profile, controller);
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
