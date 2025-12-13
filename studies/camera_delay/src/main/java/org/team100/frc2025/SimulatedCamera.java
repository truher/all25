package org.team100.frc2025;

import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.Blip24;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;

/** Publish a tag */
public class SimulatedCamera implements Runnable {
    // TODO: this should be something like 85 ms
    private static final double DELAY_S = 0.08;
    /** client instance, not the default */
    private final NetworkTableInstance m_inst;
    private final StructArrayPublisher<Blip24> m_pub;

    public SimulatedCamera() {
        m_inst = NetworkTableInstance.create();
        // This is a client just like the camera is a client.
        m_inst.setServer("localhost");
        m_inst.startClient4("SimulatedTagDetector");
        String name = "vision/0/0/blips";
        m_pub = m_inst.getStructArrayTopic(
                name, Blip24.struct).publish(PubSubOption.keepDuplicates(true));

    }

    @Override
    public void run() {
        // for now just use the clock
        // TODO: look at the ground truth
        double roll = (((Takt.actual()-0.5) % 1) - 0.5) * 2 * Math.PI;
        // camera coordinates are z-forward so roll appears there
        Transform3d t = new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, roll));
        Blip24 b = new Blip24(1, t);
        double timestampS = Takt.actual() - DELAY_S;
        long time = (long) (timestampS * 1000000.0);
        m_pub.set(new Blip24[] { b }, time);
    }

}
