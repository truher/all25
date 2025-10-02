package org.team100.lib.util;

/**
 * A wrapper that makes it easier to find all the IDs.
 * 
 * Note this mixes together PWM and Analog and DIO, which is fine, it's really
 * just for finding the numbers.
 */
public class RoboRioChannel {
    public final int channel;

    public RoboRioChannel(int channel) {
        this.channel = channel;
    }

}
