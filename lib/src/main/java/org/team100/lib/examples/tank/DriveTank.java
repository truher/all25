package org.team100.lib.examples.tank;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class DriveTank extends Command {
    private static final double SCALE = 4;

    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_rotation;
    private final TankDrive m_drive;

    public DriveTank(
            DoubleSupplier translation,
            DoubleSupplier rotation,
            TankDrive robotDrive) {
        m_translation = translation;
        m_rotation = rotation;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        // double rotscale = 1 - 0.5 * Math.abs(m_translation.getAsDouble());
        m_drive.setVelocity(m_translation.getAsDouble() * SCALE, m_rotation.getAsDouble() * 5 * SCALE);
        // m_drive.setDutyCycle(m_translation.getAsDouble() * SCALE,
        // m_rotation.getAsDouble() * rotscale);
    }
}
