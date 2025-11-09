package org.team100.lib.commands.r3.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.helper.VelocityReferenceControllerR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.ConstantReferenceR3;
import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

/**
 * Given a swerve state, drive there using only the feedback in the holonomic
 * controller.
 * 
 * There's no profile, no trajectory, just give it a point
 * you want to go to, and it will go there. It makes no attempt to
 * impose feasibility constraints or coordinate the axes, so it tries
 * to use the setpoint generator to moderate the output.
 * 
 * This is really only intended for testing.
 */
public class DriveToStateSimple extends MoveAndHold {
    private final LoggerFactory m_log;
    private final ControllerR3 m_controller;
    private final VelocitySubsystemR3 m_drive;
    private final ReferenceR3 m_reference;

    private VelocityReferenceControllerR3 m_referenceController;

    public DriveToStateSimple(
            LoggerFactory parent,
            ControllerR3 controller,
            VelocitySubsystemR3 drive,
            ModelR3 goal) {
        m_log = parent.type(this);
        m_controller = controller;
        m_drive = drive;
        m_reference = new ConstantReferenceR3(goal);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
