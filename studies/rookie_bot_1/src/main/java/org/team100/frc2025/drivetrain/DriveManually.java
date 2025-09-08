package org.team100.frc2025.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.hid.DriverControl;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveManually extends Command {
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final TankDriveSubsystem m_drive;

    public DriveManually(
            Supplier<DriverControl.Velocity> twistSupplier,
            TankDriveSubsystem robotDrive) {
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        DriverControl.Velocity input = m_twistSupplier.get();
        m_drive.setRaw(input.x(), input.y());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
