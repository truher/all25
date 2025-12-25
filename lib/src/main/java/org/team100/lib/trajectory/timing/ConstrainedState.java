package org.team100.lib.trajectory.timing;

class ConstrainedState {
    public double velocity;
    public double decel;
    public double accel;

    public ConstrainedState() {
        velocity = 100;
        decel = -100;
        accel = 100;
    }
}