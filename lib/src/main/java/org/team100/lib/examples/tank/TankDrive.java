package org.team100.lib.examples.tank;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Tank drive that uses "Arcade Drive" for control. */
public interface TankDrive extends Subsystem {
    void set(double translationSpeed, double rotSpeed);

    void stop();
}