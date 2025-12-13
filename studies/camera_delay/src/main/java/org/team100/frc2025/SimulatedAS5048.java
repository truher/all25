package org.team100.frc2025;

import org.team100.lib.coherence.Takt;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.simulation.DutyCycleSim;

/**
 * For simulation of position through the real AS5048 code.
 */
public class SimulatedAS5048 implements Runnable {

    private final DutyCycleSim m_sim;

    public SimulatedAS5048(RoboRioChannel channel) {
        m_sim = DutyCycleSim.createForChannel(channel.channel);
        m_sim.setInitialized(true);
        m_sim.setFrequency(1000);
        m_sim.setOutput(0.5);
    }

    @Override
    public void run() {
        m_sim.setOutput(Takt.actual() % 1);
    }

}
