package org.team100.lib.examples.semiauto;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.targeting.ObjectPosition24ArrayListener;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Pick a game piece from the floor, based on camera input. */
public class FloorPickSetup {
    public FloorPickSetup(
            FieldLogger.Log fieldLog,
            DriverControl control,
            SwerveDriveSubsystem drive,
            ObjectPosition24ArrayListener listener,
            SwerveController controller,
            HolonomicProfile profile) {
        new Trigger(control::button4).whileTrue(
                FloorPickSequence.get(fieldLog, drive, listener, controller, profile));
    }
}
