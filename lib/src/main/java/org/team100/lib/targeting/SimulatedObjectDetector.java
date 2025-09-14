package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.team100.lib.util.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedObjectDetector {
    private static final boolean DEBUG = true;
    /** horizontal field of view, half-angle */
    private final static double HFOV_HALF = Math.toRadians(40);
    /** vertical field of view, half-angle */
    private final static double VFOV_HALF = Math.toRadians(31.5);

    private SimulatedObjectDetector() {

    }

    /**
     * Gets the rotation to the object in the frame
     * 
     * @param simulatedObjectDetector
     * 
     * @param robotPose               Pose of the robot
     * @param notes                   field relative translation of any objects
     */
    public static List<Rotation3d> getRotations(
            Pose2d robotPose, Transform3d cameraOffset, Translation2d[] notes) {
        ArrayList<Rotation3d> list = new ArrayList<>();
        for (Translation2d note : notes) {
            if (DEBUG)
                Util.printf("note %s\n", note);
            getRotInCamera(robotPose, cameraOffset, note).ifPresent(list::add);
        }
        return list;
    }

    // package-private below for testing

    /**
     * Return the camera-relative rotation for the note, given the robot pose.
     */
    static Optional<Rotation3d> getRotInCamera(Pose2d robotPose, Transform3d cameraOffset, Translation2d note) {
        Transform3d noteInCameraCoordinates = getNoteInCameraCoordinates(
                robotPose, cameraOffset, note);
        double x = noteInCameraCoordinates.getX();
        double y = noteInCameraCoordinates.getY();
        double z = noteInCameraCoordinates.getZ();
        Rotation3d rot = new Rotation3d(VecBuilder.fill(x, 0, 0), VecBuilder.fill(x, y, z));
        if (Math.abs(rot.getY()) >= VFOV_HALF
                && Math.abs(rot.getZ()) >= HFOV_HALF) {
            if (DEBUG)
                Util.printf("out of frame\n");
            return Optional.empty();
        }
        if (DEBUG)
            Util.printf("rot %s\n", rot);
        return Optional.of(rot);
    }

    /**
     * Find the note transform relative to the camera, given the field-relative
     * robot pose and note translation
     */
    static Transform3d getNoteInCameraCoordinates(Pose2d robotPose, Transform3d cameraOffset, Translation2d note) {
        Pose2d notePose = new Pose2d(note, new Rotation2d());
        Translation2d relative = notePose.relativeTo(robotPose).getTranslation();
        return getNoteInCameraCoordinates(cameraOffset, relative);
    }

    /**
     * Given a translation on the floor relative to the robot, return the transform
     * relative to the camera.
     */
    static Transform3d getNoteInCameraCoordinates(Transform3d cameraOffset, Translation2d relative) {
        Transform3d noteInRobotCoords = new Transform3d(
                new Translation3d(relative.getX(), relative.getY(), 0),
                new Rotation3d());
        Transform3d robotInCameraCoordinates = cameraOffset.inverse();
        return robotInCameraCoordinates.plus(noteInRobotCoords);
    }
}
