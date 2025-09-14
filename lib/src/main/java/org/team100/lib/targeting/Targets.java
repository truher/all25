package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 * 
 * TODO: combine with AprilTagRobotLocalizer, extract the differences.
 */
public class Targets extends CameraReader<Rotation3d> {
    private static final boolean DEBUG = true;

    /** Ignore sights older than this. */
    private static final double MAX_SIGHT_AGE = 0.1;

    List<Translation2d> fieldRelativeTargets = new ArrayList<>();
    /** robotpose = f(takt seconds) */
    private final DoubleFunction<Pose2d> m_robotPose;

    private double m_latestTime = 0;

    public Targets(
            DoubleFunction<Pose2d> robotPose,
            String ntRootName,
            String ntValueName,
            StructBuffer<Rotation3d> buf) {
        super(ntRootName, ntValueName, buf);
        m_robotPose = robotPose;

    }

    @Override
    public void finishUpdate() {
    }

    @Override
    public void beginUpdate() {
    }

    @Override
    public void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Rotation3d[] sights) {
        m_latestTime = Takt.get();

        Pose2d robotPose = m_robotPose.apply(valueTimestamp);

        // TODO: this overwrites the whole target set with whatever one camera sees
        // TODO: instead it should merge the sights from several cameras.
        fieldRelativeTargets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose,
                cameraOffset,
                sights);
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTranslation2dArray() {
        update();
        if (m_latestTime > Takt.get() - MAX_SIGHT_AGE) {
            return fieldRelativeTargets;
        }
        return new ArrayList<>();
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTranslation2d() {
        update();
        Pose2d robotPose = m_robotPose.apply(Takt.get());
        List<Translation2d> translation2dArray = getTranslation2dArray();
        if (DEBUG)
            Util.printf("translations %d\n", translation2dArray.size());
        return ObjectPicker.closestObject(
                translation2dArray,
                robotPose);
    }
}