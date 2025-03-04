package org.team100.frc2025.Swerve;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Swerve.Auto.GoToCoralStationLeft;
import org.team100.frc2025.Swerve.Auto.GoToReefDestination;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Maker {

    TimingConstraintFactory constraints;

    LoggerFactory m_logger;

    SwerveDriveSubsystem m_drive;
    TrajectoryVisualization m_viz;
    SwerveKinodynamics m_kinodynamics;

    Trajectory100 m_clockWiseTraj;

    public Maker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        m_logger = parent.child("Maker");
        m_drive = swerve;
        constraints = new TimingConstraintFactory(kinodynamics);
        m_viz = viz;
        m_kinodynamics = kinodynamics;

    }

    public Command embark() {
        final SwerveController holonomicController = SwerveControllerFactory.ridiculous(m_logger);


        final HolonomicProfile profile = new HolonomicProfile(
                m_kinodynamics.getMaxDriveVelocityM_S(),
                m_kinodynamics.getMaxDriveAccelerationM_S2(),
                0.01, // 1 cm
                m_kinodynamics.getMaxAngleSpeedRad_S(),
                m_kinodynamics.getMaxAngleAccelRad_S2(),
                0.1); // 5 degrees
        return new Embark(m_drive, holonomicController, profile);
    }

    public Command test() {

        return new SequentialCommandGroup(
                new GoToReefDestination(
                        m_logger,
                        m_drive,
                        SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz,
                        m_kinodynamics,
                        FieldSector.IJ,
                        ReefDestination.RIGHT),
                new GoToCoralStationLeft(
                        m_logger,
                        m_drive,
                        SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz,
                        m_kinodynamics,
                        0.5),
                new GoToReefDestination(
                        m_logger,
                        m_drive,
                        SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz,
                        m_kinodynamics,
                        FieldSector.KL,
                        ReefDestination.LEFT),
                new GoToCoralStationLeft(
                        m_logger,
                        m_drive,
                        SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz,
                        m_kinodynamics,
                        0),
                new GoToReefDestination(
                        m_logger,
                        m_drive,
                        SwerveControllerFactory.fieldRelativeGoodPIDF(m_logger),
                        m_viz,
                        m_kinodynamics,
                        FieldSector.KL,
                        ReefDestination.RIGHT));

    }

}