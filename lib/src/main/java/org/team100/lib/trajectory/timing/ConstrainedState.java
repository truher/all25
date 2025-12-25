package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

class ConstrainedState {
    public final Pose2dWithMotion state;
    public final double distance;
    public double velocity;
    public double decel;
    public double accel;

    public ConstrainedState(Pose2dWithMotion state, double distance) {
        this.state = state;
        this.distance = distance;
        velocity = 100;
        decel = -100;
        accel = 100;
    }
}