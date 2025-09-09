package org.team100.lib.examples.tank;

import java.util.function.Supplier;

import org.team100.lib.hid.DriverControl;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class DriveTank extends Command {
    private static final double SCALE = 0.4;
    private static final double ROT_SCALE = 0.3;

    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final TankDrive m_drive;

    public DriveTank(
            Supplier<DriverControl.Velocity> twistSupplier,
            TankDrive robotDrive) {
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        DriverControl.Velocity input = m_twistSupplier.get();
        m_drive.set(input.x() * SCALE, input.y() * ROT_SCALE);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
