package org.team100.frc2025.robot;

import java.util.List;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.timing.TrajectoryFactory;

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
        LoggerFactory autoLog = Logging.instance().rootLogger.name("Auton");

        final ProfileSE2 profile = HolonomicProfileFactory.currentLimitedExponential(1, 2, 4,
                machinery.m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                machinery.m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                5);
        final FullStateControllerSE2 controller = ControllerFactorySE2
                .auto2025LooseTolerance(autoLog);
        List<TimingConstraint> constraints = new TimingConstraintFactory(machinery.m_swerveKinodynamics)
                .medium(autoLog);
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        final TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);

        // WARNING! The glass widget will override the default, so check it!
        // Run the auto in pre-match testing!
        m_autonChooser.addAsDefault("Lollipop",
                new AnnotatedCommand(
                        new LolipopAuto(autoLog, machinery, profile, controller, planner).get(), null, null));

        DriveAndScore driveAndScore = new DriveAndScore(autoLog, machinery, profile, controller);
        m_autonChooser.add("Coral 1 left",
                new AnnotatedCommand(driveAndScore.get(ScoringLevel.L4, ReefPoint.J), null, null));
        m_autonChooser.add("Coral 1 mid",
                new AnnotatedCommand(driveAndScore.get(ScoringLevel.L4, ReefPoint.H), null, null));
        m_autonChooser.add("Coral 1 right",
                new AnnotatedCommand(driveAndScore.get(ScoringLevel.L4, ReefPoint.F), null, null));

        Auton auton = new Auton(autoLog, machinery, profile, controller);
        m_autonChooser.add("Left Preload Only", new AnnotatedCommand(
                auton.leftPreloadOnly(), null, null));
        m_autonChooser.add("Center Preload Only", new AnnotatedCommand(
                auton.centerPreloadOnly(), null, null));
        m_autonChooser.add("Right Preload Only", new AnnotatedCommand(
                auton.rightPreloadOnly(), null, null));
        m_autonChooser.add("Left Three Coral", new AnnotatedCommand(
                auton.left(), null, null));
        m_autonChooser.add("Right Three Coral", new AnnotatedCommand(
                auton.right(), null, null));
    }

    public Command get() {
        return m_autonChooser.get().command();
    }

    public void close() {
        m_autonChooser.close();
    }

}
