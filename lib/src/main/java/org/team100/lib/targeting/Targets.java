package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.TrailingHistory;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 */
public class Targets extends CameraReader<Rotation3d> {
    private static final boolean DEBUG = true;

    /**
     * Ignore incoming sights older than this, because they're stale.
     * This shouldn't happen if the camera is working well.
     */
    private static final double MAX_SIGHT_AGE = 0.2;
    /** Forget sights older than this. */
    private static final double HISTORY_DURATION = 1.0;

    /** robotpose = f(takt seconds); the PoseEstimator does this. */
    private final DoubleFunction<Pose2d> m_robotPose;
    private final TrailingHistory<Translation2d> m_targets;
    private final StructBuffer<Rotation3d> m_buf;
    /**
     * this is just an indicator for whether we've run update().
     * It's null after the cache refresh and true after the update runs.
     */
    private final CotemporalCache<Boolean> m_hot;

    public Targets(
            DoubleFunction<Pose2d> robotPose,
            String ntRootName,
            String ntValueName) {
        super(ntRootName, ntValueName);
        m_robotPose = robotPose;
        m_targets = new TrailingHistory<>(HISTORY_DURATION);
        m_buf = StructBuffer.create(Rotation3d.struct);
        m_hot = Cache.of(() -> {
            update();
            return true;
        });
    }

    @Override
    public StructBuffer<Rotation3d> getBuffer() {
        return m_buf;
    }

    @Override
    public void beginUpdate() {
    }

    @Override
    public void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Rotation3d[] sights) {
        double age = Takt.get() - valueTimestamp;
        if (age > MAX_SIGHT_AGE) {
            if (DEBUG)
                Util.warnf("ignoring stale sight %f\n", age);
            return;
        }
        Pose2d robotPose = m_robotPose.apply(valueTimestamp);
        m_targets.addAll(
                valueTimestamp,
                TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraOffset,
                        sights));
    }

    @Override
    public void finishUpdate() {
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTargets() {
        return m_targets.getAll();
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTarget() {
        Pose2d robotPose = m_robotPose.apply(Takt.get());
        List<Translation2d> targets = getTargets();
        if (DEBUG)
            Util.printf("translations %d\n", targets.size());
        return ObjectPicker.closestObject(targets, robotPose);
    }
}