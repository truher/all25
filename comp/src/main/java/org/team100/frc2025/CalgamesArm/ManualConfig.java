package org.team100.frc2025.CalgamesArm;

import java.util.function.Supplier;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.Config;

import edu.wpi.first.wpilibj2.command.Command;

/** Use the operator control to "fly" the arm around in config space. */
public class ManualConfig extends Command {
    // TODO: some reasonable scale here
    private static final double SCALE = 0.01;
    private final Supplier<DriverControl.Velocity> m_input;
    private final CalgamesMech m_subsystem;

    private Config m_config;

    public ManualConfig(
            Supplier<DriverControl.Velocity> input,
            CalgamesMech subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_config = m_subsystem.getConfig();
    }

    @Override
    public void execute() {
        DriverControl.Velocity input = m_input.get();
        m_config = new Config(
                m_config.shoulderHeight() + input.x() * SCALE,
                m_config.shoulderAngle() + input.y() * SCALE,
                m_config.wristAngle() + input.theta() * SCALE);
        m_subsystem.set(m_config);
    }
}
