package org.team100.lib.examples.tank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Differential ("tank") drive. */
public interface TankDrive extends Subsystem {
    
    /** Set the drive velocity. */
    default Command driveWithVelocity(double translationM_S, double rotationRad_s) {
        return run(() -> setVelocity(translationM_S, rotationRad_s));
    }

    /** Use arcade drive to set duty cycle directly. */
    void setDutyCycle(double translationSpeed, double rotSpeed);

    /** Use inverse kinematics to set wheel speeds. */
    void setVelocity(double translationM_S, double rotationRad_S);

    void stop();
}