package org.team100.frc2025.Swerve.SemiAuto;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.ReefAproach;
import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.PoseSet;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class Navigator extends Command implements Planner2025 {
    public static class Log {
        public final Pose2dLogger m_log_goal;
        public final FieldRelativeVelocityLogger m_log_chassis_speeds;
        public final DoubleLogger m_log_THETA_ERROR;
        public final BooleanLogger m_log_FINSIHED;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("Navigator");
            m_log_goal = log.pose2dLogger(Level.TRACE, "goal");
            m_log_chassis_speeds = log.fieldRelativeVelocityLogger(Level.TRACE, "speed");
            m_log_THETA_ERROR = log.doubleLogger(Level.TRACE, "THETA ERROR");
            m_log_FINSIHED = log.booleanLogger(Level.TRACE, "FINSIHED");
        }
    }

    public final Log m_log;
    protected final SwerveDriveSubsystem m_drive;
    protected final TrajectoryFollower m_controller;
    private Pose2d m_goal = new Pose2d();
    protected final TrajectoryVisualization m_viz;

    TimingConstraintFactory m_constraints;

    public Navigator(
            LoggerFactory parent,
            SwerveDriveSubsystem robotDrive,
            TrajectoryFollower controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        m_log = new Log(parent);
        m_drive = robotDrive;
        m_controller = controller;
        m_viz = viz;
        m_constraints = new TimingConstraintFactory(kinodynamics);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        throw new UnsupportedOperationException("subclasses must implement initialize()");
    }

    @Override
    public final void execute() {

        Pose2d currentPose = m_drive.getPose();

        Rotation2d angleToReef = FieldConstants.angleToReefCenter(currentPose);
        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d thetaError = angleToReef.minus(currentHeading);

        // m_controller.setThetaError(thetaError);
        FieldRelativeVelocity output = m_controller.update(m_drive.getState());

        m_drive.driveInFieldCoordsVerbatim(output);

        m_log.m_log_chassis_speeds.log(() -> output);
        double thetaErrorRad = m_goal.getRotation().getRadians()
                - m_drive.getPose().getRotation().getRadians();
        m_log.m_log_THETA_ERROR.log(() -> thetaErrorRad);
        m_log.m_log_FINSIHED.log(() -> false);
    }

    @Override
    public final void end(boolean interrupted) {
        m_log.m_log_FINSIHED.log(() -> true);
        m_drive.stop();
        m_viz.clear();
    }

    @Override
    public final boolean isFinished() {
        // return m_controller.isDone();
        return false;
    }

    public static PoseSet addRobotPose(Pose2d currPose, List<Pose2d> waypoints, List<Rotation2d> headings) {
        Translation2d currTranslation = currPose.getTranslation();
        Translation2d firstWaypoint = waypoints.get(0).getTranslation();
        Rotation2d initialSpline = firstWaypoint.minus(currTranslation).getAngle();
        Pose2d initialWaypoint = new Pose2d(currTranslation, initialSpline);
        Rotation2d initialHeading = currPose.getRotation();

        List<Pose2d> waypointsWithPose = new ArrayList<>();
        List<Rotation2d> headingsWithPose = new ArrayList<>();

        waypointsWithPose.addAll(waypoints);
        headingsWithPose.addAll(headings);

        waypointsWithPose.add(0, initialWaypoint);
        headingsWithPose.add(0, initialHeading);

        return new PoseSet(waypointsWithPose, headingsWithPose);
    }

    public static PoseSet addRobotPose(Pose2d currPose, List<Pose2d> waypoints, List<Rotation2d> headings,
            Rotation2d initialSpline) {
        Translation2d currTranslation = currPose.getTranslation();
        Translation2d firstWaypoint = waypoints.get(0).getTranslation();
        // Rotation2d initialSpline = firstWaypoint.minus(currTranslation).getAngle();
        Pose2d initialWaypoint = new Pose2d(currTranslation, initialSpline);
        Rotation2d initialHeading = currPose.getRotation();

        List<Pose2d> waypointsWithPose = new ArrayList<>();
        List<Rotation2d> headingsWithPose = new ArrayList<>();
        ;

        waypointsWithPose.addAll(waypoints);
        headingsWithPose.addAll(headings);

        waypointsWithPose.add(0, initialWaypoint);
        headingsWithPose.add(0, initialHeading);

        return new PoseSet(waypointsWithPose, headingsWithPose);
    }

    public Rotation2d calculateInitialSpline(Translation2d targetPoint, Translation2d currTranslation, Translation2d vectorFromCenterToRobot, ReefAproach approach, double magicNumber){

        double distanceToReef = FieldConstants.getDistanceToReefCenter(currTranslation);

        Translation2d translationToTarget = targetPoint.minus(currTranslation);
        
        Rotation2d rotationAngle = new Rotation2d();

        switch(approach){
            case CCW:
                rotationAngle = Rotation2d.fromDegrees(90);
                break;
            case CW:
                rotationAngle = Rotation2d.fromDegrees(-90);
                break;
        }


        Translation2d tangentVector = vectorFromCenterToRobot.rotateBy(rotationAngle);


        Translation2d tangentVectorAdjusted = tangentVector.times((1/distanceToReef) * magicNumber); // MAGIC NUMBER is a MAGIC NUMBER

        Translation2d finalVector = translationToTarget.plus(tangentVectorAdjusted);

        Rotation2d finalAngle =  finalVector.getAngle();

        return finalAngle;
    }

    public static Rotation2d calculateInitialSpline(Translation2d targetPoint, Translation2d currTranslation) {
        Rotation2d initialSpline = targetPoint.minus(currTranslation).getAngle();
        return initialSpline;

    }

}
