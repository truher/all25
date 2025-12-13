package org.team100.lib.network;

import java.util.function.ObjDoubleConsumer;

import org.team100.lib.localization.Blip24;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.StructBuffer;

/** Listen to raw tag input from the cameras, for testing. */
public class RawTags extends CameraReader<Blip24> {
    private final ObjDoubleConsumer<Transform3d> m_sink;

    public RawTags(LoggerFactory parent, ObjDoubleConsumer<Transform3d> sink) {
        super(parent, "vision", "blips", StructBuffer.create(Blip24.struct));
        m_sink = sink;
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Blip24[] value) {
        for (Blip24 b : value) {
            m_sink.accept(b.blipToTransform(), valueTimestamp);
        }
    }

}
