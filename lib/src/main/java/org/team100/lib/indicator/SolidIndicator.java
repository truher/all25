package org.team100.lib.indicator;

import java.util.HashMap;
import java.util.Map;

import org.team100.lib.coherence.Takt;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An LED indicator that is all the same color, optionally blinking.
 * 
 * Blink rate depends on the signal urgency:
 * 
 * * low-urgency signals are steady.
 * * moderately-urgent signals blink at 2.5 Hz at 50% duty cycle.
 * * more-urgent signals blink at 5 Hz at 50% duty cycle.
 * 
 * Resources regarding signal design:
 * 
 * https://www.denix.osd.mil/soh/denix-files/sites/21/2016/03/02_MIL-STD-1472F-Human-Engineering.pdf
 * https://www.faa.gov/regulations_policies/rulemaking/committees/documents/media/TAEas-wcal-04232002.pdf
 */
public class SolidIndicator {
    private static final double DUTY_CYCLE = 0.5;
    private static final double STEADY_HZ = 0;
    private static final double SLOW_HZ = 1;
    private static final double FAST_HZ = 4;

    private final AddressableLED m_led;
    private final int m_length;
    private final AddressableLEDBuffer m_black;
    private final Map<Color, AddressableLEDBuffer> m_buffers;
    private AddressableLEDBuffer m_active;
    private double m_freq;

    public SolidIndicator(RoboRioChannel channel, int length) {
        m_led = new AddressableLED(channel.channel);
        m_length = length;
        m_buffers = new HashMap<>();
        m_black = fill(Color.kBlack);
        m_active = m_black;
        m_led.setLength(length);
        m_led.start();
    }

    public void steady(Color color) {
        setColor(color);
        setBlinkRate(STEADY_HZ);
    }

    public void slow(Color color) {
        setColor(color);
        setBlinkRate(SLOW_HZ);
    }

    public void fast(Color color) {
        setColor(color);
        setBlinkRate(FAST_HZ);
    }

    public void periodic() {
        if (m_freq < 0.01 || m_freq > 10) {
            m_led.setData(m_active);
            return;
        }
        double period = 1 / m_freq;
        double phase = (Takt.get() % period) / period;
        if (phase < DUTY_CYCLE) {
            m_led.setData(m_active);
        } else {
            m_led.setData(m_black);
        }
    }

    /////////////////////////////////////////////////////////////

    private void setColor(Color color) {
        m_active = m_buffers.computeIfAbsent(color, this::fill);
    }

    private void setBlinkRate(double freq) {
        m_freq = freq;
    }

    private AddressableLEDBuffer fill(Color color) {
        AddressableLEDBuffer buf = new AddressableLEDBuffer(m_length);
        for (int i = 0; i < m_length; ++i) {
            buf.setLED(i, color);
        }
        return buf;
    }

}
