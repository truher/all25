package org.team100.lib.commands.drivetrain.mecanum;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.manual.FieldRelativeDriver;
import org.team100.lib.examples.mecanum.MecanumDrive100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualMecanum extends Command {
    private final Supplier<Velocity> m_velocity;
    private final double m_maxV;
    private final double m_maxOmega;
    private final SwerveLimiter m_limiter;
    private final MecanumDrive100 m_drive;

    public ManualMecanum(
            Supplier<Velocity> velocity,
            double maxV,
            double maxOmega,
            SwerveLimiter limiter,
            MecanumDrive100 drive) {
        m_velocity = velocity;
        m_maxV = maxV;
        m_maxOmega = maxOmega;
        m_limiter = limiter;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_limiter.updateSetpoint(m_drive.getState().velocity());
    }

    @Override
    public void execute() {
        Velocity input = m_velocity.get();
        // clip the input to the unit circle
        Velocity clipped = input.clip(1.0);
        GlobalVelocityR3 scaled = FieldRelativeDriver.scale(clipped, m_maxV, m_maxOmega);
        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

}
