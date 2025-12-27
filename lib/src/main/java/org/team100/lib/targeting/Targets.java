package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.CentroidR2;
import org.team100.lib.geometry.NearR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.CoalescingCollection;
import org.team100.lib.util.TrailingHistory;

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
    /** Targets closer than this to each other are combined */
    private static final double NEARNESS_THRESHOLD = 0.15;

    public final DoubleArrayLogger m_log_allTargets;
    public final DoubleArrayLogger m_log_coalescedTargets;

    /** state = f(takt seconds) from history. */
    private final DoubleFunction<ModelSE2> m_history;
    /** Accumulation of targets we see; this is really for logging only. */
    private final TrailingHistory<Translation2d> m_allTargets;
    /** Coalesced targets */
    private final CoalescingCollection<Translation2d> m_targets;
    /** Side effect mutates targets. */
    private final SideEffect m_vision;
    private final IntLogger m_log_historySize;
    private final DoubleLogger m_log_age;
    private final DoubleLogger m_log_poseTimestamp;

    public Targets(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            DoubleFunction<ModelSE2> history) {
        super(parent, "objectVision", "Rotation3d", StructBuffer.create(Rotation3d.struct));
        LoggerFactory log = parent.type(this);
        m_log_historySize = log.intLogger(Level.TRACE, "history size");
        m_log_allTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "all targets");
        m_log_coalescedTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "coalesced targets");
        m_log_age = log.doubleLogger(Level.TRACE, "target age");
        m_log_poseTimestamp = log.doubleLogger(Level.TRACE, "pose timestamp");
        m_history = history;
        m_allTargets = new TrailingHistory<>(HISTORY_DURATION);
        m_targets = new CoalescingCollection<>(
                new TrailingHistory<>(HISTORY_DURATION),
                new NearR2(NEARNESS_THRESHOLD),
                new CentroidR2());
        m_vision = Cache.ofSideEffect(this::update);
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Rotation3d[] sights) {
        double age = Takt.get() - valueTimestamp;
        m_log_age.log(() -> age);
        if (age > MAX_SIGHT_AGE) {
            if (DEBUG) {
                System.out.printf("WARNING: ignoring stale sight %f %f\n", Takt.get(), valueTimestamp);
            }
            return;
        }
        // ALERT!
        //
        // Vasili added this while testing target interception, but I have no idea why.
        // It breaks all the simulations. The effect is to make the received sight
        // appear as if it were from further in the past than the timestamp says it is,
        // which would be required if there were delay (a lot of delay) not included
        // in the timestamp.
        //
        // TODO: explore the issue of time synchronization in more depth.
        //
        // TODO: maybe add this extra delay to the python code; maybe the computation
        // there is wrong somehow.
        //
        // double SOME_SORT_OF_OFFSET = 0.05;
        double SOME_SORT_OF_OFFSET = 0.0;
        double poseTimestamp = valueTimestamp - SOME_SORT_OF_OFFSET;
        m_log_poseTimestamp.log(() -> poseTimestamp);
        Pose2d robotPose = m_history.apply(poseTimestamp).pose();

        // Tranform sights to field targets.
        List<Translation2d> targets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                robotPose, cameraOffset, sights);
        if (DEBUG) {
            System.out.printf("targets %d\n", targets.size());
        }

        m_allTargets.addAll(valueTimestamp, targets);
        m_targets.addAll(valueTimestamp, targets);
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
        if (DEBUG) {
            System.out.printf("translations %d\n", targets.size());
        }
        return ObjectPicker.closestObject(targets, robotPose);
    }

    public void periodic() {
        // show the closest target we can see on the field2d widget.
        // getClosestTarget().ifPresent(
        // x -> m_field_log.m_log_target.log(
        // () -> new double[] { x.getX(), x.getY(), 0 }));

        // Show coalesced targets on the field2d widget.
        m_log_coalescedTargets.log(
                () -> getTargets().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        // Also show *all* the raw sightings.
        m_log_allTargets.log(
                () -> m_allTargets.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        m_log_historySize.log(() -> m_targets.size());
    }
}