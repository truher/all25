package org.team100.lib.hid;

/**
 * This represents driver's velocity command, usually mapped to three axes in
 * the control, so the ranges are [-1,1]
 */
public record Velocity(double x, double y, double theta) {

    /**
     * Clip the translational velocity to the unit circle.
     * 
     * This means that there will be no stick response outside the circle, but the
     * inside will be unchanged.
     * 
     * The argument for clipping is that it leaves the response inside the circle
     * alone: with squashing, the diagonals are more sensitive.
     * 
     * The argument for squashing is that it preserves relative response in
     * the corners: with clipping, going full speed diagonally and then "slowing
     * down a little" will do nothing.
     * 
     * If you'd like to avoid clipping, then squash the input upstream, in the
     * control class.
     */
    public Velocity clip(double maxMagnitude) {
        double hyp = Math.hypot(x(), y());
        if (hyp < 1e-12)
            return this;
        double clamped = Math.min(hyp, maxMagnitude);
        double ratio = clamped / hyp;
        return new Velocity(ratio * x(), ratio * y(), theta());
    }
}