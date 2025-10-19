package org.team100.lib.examples.mecanum;

import org.team100.lib.commands.drivetrain.manual.FieldRelativeDriver;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Mecanum drive. */
public interface MecanumDrive extends Subsystem {
    

    /** Set the drive velocity. */
    default Command driveWithVelocity(Velocity v) {
        return run(() -> setVelocity(FieldRelativeDriver.scale(v, 1, 1)));
    }

    /** Use inverse kinematics to set wheel speeds. */
    void setVelocity(GlobalVelocityR3 input);

    void stop();
}