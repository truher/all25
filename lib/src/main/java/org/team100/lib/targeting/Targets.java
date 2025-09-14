package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 * 
 * TODO: combine with AprilTagRobotLocalizer, extract the differences.
 */
public class Targets {
    private static final boolean DEBUG = true;

    /** Ignore sights older than this. */
    private static final double MAX_SIGHT_AGE = 0.1;

    private StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);
    List<Translation2d> fieldRelativeTargets = new ArrayList<>();
    private final DoubleFunction<Pose2d> m_robotPose;
    private final NetworkTableListenerPoller m_poller;

    private double latestTime = 0;

    private final String m_ntValueName;

    public Targets(
            DoubleFunction<Pose2d> robotPose, String ntValueName) {
        m_robotPose = robotPose;
        m_ntValueName = ntValueName;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { "objectVision" },
                        PubSubOption.keepDuplicates(true)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
    }

    /** Read pending camera input, transform to field-relative targets. */
    public void update() {

        for (NetworkTableEvent e : m_poller.readQueue()) {
            ValueEventData valueEventData = e.valueData;
            NetworkTableValue ntValue = valueEventData.value;
            String name = valueEventData.getTopic().getName();
            if (DEBUG)
                Util.printf("poll %s\n", name);
            String[] fields = name.split("/");
            if (fields.length != 4) {
                Util.warnf("weird event name: %s\n", name);
                continue;
            }
            if (fields[2].equals("fps")) {
                // FPS is not used by the robot
            } else if (fields[2].equals("latency")) {
                // latency is not used by the robot
            } else if (fields[3].equals("Rotation3d")) {
                // e.g. "blips" or "Rotation3d"
                if (DEBUG)
                    Util.printf("found value\n");
                // decode the way StructArrayEntryImpl does
                byte[] b = ntValue.getRaw();
                if (b.length == 0) {
                    // this should never happen, but it does, very occasionally.
                    continue;
                }



                

                // object sights are x-ahead WPI coordinates, not z-ahead camera coordinates.
                Rotation3d[] sights;
                try {
                    synchronized (m_buf) {
                        sights = m_buf.readArray(b);
                        latestTime = Takt.get();
                    }
                } catch (RuntimeException ex) {
                    Util.warnf("decoding failed for name: %s\n", name);
                    return;
                }
                Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
                if (DEBUG)
                    Util.printf("camera %s offset %s\n", fields[1], cameraInRobotCoordinates);
                Pose2d robotPose = m_robotPose.apply(ntValue.getServerTime() / 1000000.0);
                // TODO: this overwrites the whole target set with whatever one camera sees
                // TODO: instead it should merge the sights from several cameras.
                fieldRelativeTargets = TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraInRobotCoordinates,
                        sights);
            } else {
                Util.warn("object weird vision update key: " + name);
            }
        }
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTranslation2dArray() {
        update();
        if (latestTime > Takt.get() - MAX_SIGHT_AGE) {
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