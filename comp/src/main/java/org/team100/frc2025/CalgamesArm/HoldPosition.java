package org.team100.frc2025.CalgamesArm;

import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Record the current pose at initialization time and hold that pose,
 * motionless, forever.
 */
public class HoldPosition extends Command {
    private final CalgamesMech m_subsystem;

    private ModelR3 m_state;

    public HoldPosition(CalgamesMech subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // motionless at the current location
        m_state = new ModelR3(m_subsystem.getState().pose());
    }

    @Override
    public void execute() {
        m_subsystem.set(m_state.control());
        // m_subsystem.stop();
    }

}
