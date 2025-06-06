package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PostDropAndReadyFunnel;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.WaitForBooleanTrue;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.RunFunnel;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.RunCoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.CoralStation;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.ParallelRaceGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Coral2AutoRightNewNewSim extends SequentialCommandGroup100 {

    public Coral2AutoRightNewNewSim(
            LoggerFactory logger,
            Wrist2 wrist, Elevator elevator,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        super(logger, "Coral2Auto");

        Embark embarkToI = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.EF,
                ReefDestination.RIGHT,
                () -> ScoringPosition.L4, ReefPoint.F, true);

        Embark embarkToK = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.CD,
                ReefDestination.RIGHT,
                () -> ScoringPosition.L4, ReefPoint.D, true);

        Embark embarkToL = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.CD,
                ReefDestination.LEFT,
                () -> ScoringPosition.L4, ReefPoint.C, true);

        GoToCoralStation goToStation1stTime = new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics,
                                        CoralStation.Right, 0.5, true);

        GoToCoralStation goToStation2ndTime = new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics,
                                        CoralStation.Right, 0.5, true);

        PrePlaceCoralL4 prePlaceCoralL4I = new PrePlaceCoralL4(wrist, elevator, tunnel, 47, true);

        PrePlaceCoralL4 prePlaceCoralL4K = new PrePlaceCoralL4(wrist, elevator,tunnel , 47, true);

        PrePlaceCoralL4 prePlaceCoralL4L = new PrePlaceCoralL4(wrist, elevator, tunnel, 47, true);

       
        // PostDropAndReadyFunnel postDropAndReadyFunnelFromI = new PostDropAndReadyFunnel(wrist, elevator, 10, () -> false);

        PostDropAndReadyFunnel postDropAndReadyFunnelFromI = new PostDropAndReadyFunnel(wrist, elevator, 10, (goToStation1stTime::isDone));

        PostDropAndReadyFunnel postDropAndReadyFunnelFromK = new PostDropAndReadyFunnel(wrist, elevator, 10, goToStation2ndTime::isDone);



        addCommands(

                // Go to peg I and pre place preload
                new ParallelDeadlineGroup100(
                        m_logger,
                        "embark1",
                        new DeadlineForEmbarkAndPrePlace(embarkToI::isDone, () -> true),
                        embarkToI,
                        new SequentialCommandGroup100(logger, "handoff then place",
                                new ParallelRaceGroup100(m_logger, "handoff",
                                    new WaitCommand(0.5),
                                    new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                                ),
                                new SetWrist(wrist, 0.4, false),
                                prePlaceCoralL4I)),

                // Place preload and reload

                new ParallelDeadlineGroup100(
                        logger,
                        "drive away",
                        postDropAndReadyFunnelFromI,
                        new SequentialCommandGroup100(
                                logger,
                                "wait then drive away to pick up",
                                new WaitForBooleanTrue(() -> true),
                                new ParallelDeadlineGroup100(
                                        logger,
                                        "Pick up",
                                        goToStation1stTime,
                                        new RunFunnel(funnel),
                                        new RunCoralTunnel(tunnel, 1))
                        )

                ),

                //Go to peg K and pre place
 
                new ParallelDeadlineGroup100(
                    m_logger,
                    "embark1",
                    new DeadlineForEmbarkAndPrePlace(embarkToK::isDone, () -> true),
                    embarkToK,
                    new SequentialCommandGroup100(logger, "handoff then place",
                            new ParallelRaceGroup100(m_logger, "handoff",
                                new WaitCommand(0.5),
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                            ),
                            new SetWrist(wrist, 0.4, false),
                            prePlaceCoralL4K)),

                // Place coral and reload

                new ParallelDeadlineGroup100(
                        logger,
                        "drive away",
                        postDropAndReadyFunnelFromK,
                        new SequentialCommandGroup100(
                                logger,
                                "wait then drive away to pick up",
                                new WaitForBooleanTrue(() -> true),
                                new ParallelDeadlineGroup100(
                                        logger,
                                        "Pick up",
                                        goToStation2ndTime,
                                        new RunFunnel(funnel),
                                        new RunCoralTunnel(tunnel, 1)))

                ),

                //Go to peg L and pre place
 
                new ParallelDeadlineGroup100(
                        m_logger,
                        "embark1",
                        new DeadlineForEmbarkAndPrePlace(embarkToL::isDone, () -> true),
                        embarkToL,
                        new SequentialCommandGroup100(logger, "handoff then place",
                                new ParallelRaceGroup100(m_logger, "handoff",
                                        new WaitCommand(0.5),
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                                ),
                                new SetWrist(wrist, 0.4, false),
                                prePlaceCoralL4L)),


                new PostDropCoralL4(wrist, elevator, 10)

        

        );




        // addCommands(
        //     embarkToI,
        //     goToStation1stTime,
        //     embarkToK,
        //     goToStation2ndTime,
        //     embarkToL
        // );
        

        

    }
}
