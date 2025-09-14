package org.team100.lib.network;

import java.util.EnumSet;

import org.team100.lib.config.Camera;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Reads camera input from network tables, which is always a StructArray.
 * 
 * @param T payload type
 */
public abstract class CameraReader<T> {
    private static final boolean DEBUG = false;
    /**
     * Five cameras, 50hz each => 250 hz of updates. Rio runs at 50 hz, so there
     * should be five messages waiting for us each cycle.
     */
    private static final int QUEUE_DEPTH = 10;

    /** e.g. "blips" or "Rotation3d" */
    private final String m_ntValueName;
    /** Deserializer */
    private final StructBuffer<T> m_buf;

    private final NetworkTableListenerPoller m_poller;

    public CameraReader(
            String ntRootName,
            String ntValueName,
            StructBuffer<T> buf) {
        m_ntValueName = ntValueName;
        m_buf = buf;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { ntRootName },
                        PubSubOption.keepDuplicates(true),
                        PubSubOption.pollStorage(QUEUE_DEPTH)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
    }

    /**
     * Read queued network input, and give it to the consumers.
     * 
     * This runs once per cycle, in SwerveDriveSubsystem.update() which is called by
     * Memo.updateAll(), which runs in Robot.robotPeriodic().
     */
    public void update() {
        beginUpdate();
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
            // key is "rootName/cameraId/cameraNumber/valueName"
            String cameraId = fields[1];
            if (!fields[3].equals(m_ntValueName)) {
                Util.warn("weird key: " + name);
                continue;
            }
            if (DEBUG)
                Util.printf("found value\n");
            // decode the way StructArrayEntryImpl does
            byte[] valueBytes = ntValue.getRaw();
            if (valueBytes.length == 0) {
                // this should never happen, but it does, very occasionally.
                continue;
            }
            T[] valueArray;
            try {
                valueArray = m_buf.readArray(valueBytes);
            } catch (RuntimeException ex) {
                Util.warnf("decoding failed for name: %s\n", name);
                continue;
            }

            // Robot-to-camera, offset from Camera.java
            // in tests this offset is identity.
            Transform3d cameraOffset = Camera.get(cameraId).getOffset();
            if (DEBUG)
                Util.printf("camera %s offset %s\n", cameraId, cameraOffset);

            // server time is in microseconds
            // https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#timestamps
            double valueTimestamp = ntValue.getServerTime() / 1000000.0;

            perValue(cameraOffset, valueTimestamp, valueArray);
        }
        finishUpdate();
    }

    /** Called when update() starts. */
    public abstract void beginUpdate();

    /**
     * Called for each StructArray received.
     * 
     * @param cameraOffset   camera pose in robot coordinates
     * @param valueTimestamp network server time in seconds
     * @param valueArray     payload array
     */
    public abstract void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            T[] value);

    /** Called when update() ends. */
    public abstract void finishUpdate();

}
