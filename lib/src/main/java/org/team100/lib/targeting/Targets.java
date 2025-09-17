package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
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
    private static final boolean DEBUG = false;

    /**
     * Ignore incoming sights older than this, because they're stale.
     * This shouldn't happen if the camera is working well.
     */
    private static final double MAX_SIGHT_AGE = 0.2;
    /** Forget sights older than this. */
    private static final double HISTORY_DURATION = 1.0;

    private final FieldLogger.Log m_field_log;

    /** state = f(takt seconds) from history. */
    private final DoubleFunction<SwerveModel> m_history;
    /** Accumulation of targets we see. */
    private final TrailingHistory<Translation2d> m_targets;
    /** Side effect mutates targets. */
    private final SideEffect m_vision;

    public Targets(
            FieldLogger.Log fieldLogger,
            DoubleFunction<SwerveModel> history) {
        super(
                "objectVision",
                "Rotation3d",
                StructBuffer.create(Rotation3d.struct));
        m_field_log = fieldLogger;
        m_history = history;
        m_targets = new TrailingHistory<>(HISTORY_DURATION);
        m_vision = Cache.ofSideEffect(this::update);
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Rotation3d[] sights) {
        double age = Takt.get() - valueTimestamp;
        if (age > MAX_SIGHT_AGE) {
            if (DEBUG)
                Util.warnf("ignoring stale sight %f %f\n",
                        Takt.get(), valueTimestamp);
            return;
        }
        Pose2d robotPose = m_history.apply(valueTimestamp).pose();
        m_targets.addAll(
                valueTimestamp,
                TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraOffset,
                        sights));
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTargets() {
        m_vision.run();
        return m_targets.getAll();
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTarget() {
        Pose2d robotPose = m_history.apply(Takt.get()).pose();
        List<Translation2d> targets = getTargets();
        if (DEBUG)
            Util.printf("translations %d\n", targets.size());
        return ObjectPicker.closestObject(targets, robotPose);
    }

    public void periodic() {
        // show the closest target we can see on the field2d widget.
        getClosestTarget().ifPresent(
                x -> m_field_log.m_log_target.log(
                        () -> new double[] { x.getX(), x.getY(), 0 }));
    }
}