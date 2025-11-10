package org.team100.lib.subsystems.r3.commands;

import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stop the drivetrain.
 */
public class DriveStop extends Command {
    private final VelocitySubsystemR3 m_drive;

    public DriveStop(VelocitySubsystemR3 robotDrive) {
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.stop();
    }
}
