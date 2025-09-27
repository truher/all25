package org.team100.frc2025.CalgamesArm;

import org.team100.lib.commands.r3.SubsystemR3;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Record the current pose at initialization time and hold that pose,
 * motionless, forever.
 */
public class HoldPosition extends Command {
    private final SubsystemR3 m_subsystem;

    private SwerveModel m_state;

    public HoldPosition(SubsystemR3 subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // motionless at the current location
        m_state = new SwerveModel(m_subsystem.getState().pose());
    }

    @Override
    public void execute() {
        m_subsystem.set(m_state.control());
    }

}
