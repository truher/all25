package org.team100.lib.examples.tank;

import java.util.function.Supplier;

import org.team100.lib.hid.Velocity;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class DriveTank extends Command {
    private static final double SCALE = 1;
    private static final double ROT_SCALE = 1;

    private final Supplier<Velocity> m_twistSupplier;
    private final TankDrive m_drive;

    public DriveTank(
            Supplier<Velocity> twistSupplier,
            TankDrive robotDrive) {
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Velocity input = m_twistSupplier.get();
        double rotscale = 1 - Math.abs(input.x());
        m_drive.set(input.x() * SCALE, input.y() * rotscale);
    }
}
