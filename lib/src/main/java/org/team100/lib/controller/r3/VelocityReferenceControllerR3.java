package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR3Logger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

/**
 * Actuates a velocity subsystem based on a reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class VelocityReferenceControllerR3 {
    private static final boolean DEBUG = false;

    private final VelocitySubsystemR3 m_subsystem;
    private final ControllerR3 m_controller;
    private final ReferenceR3 m_reference;
    private final ModelR3Logger m_log_measurement;
    private final ModelR3Logger m_log_current;
    private final ControlR3Logger m_log_next;
    private final ModelR3Logger m_log_error;

    /**
     * Initializes the reference with the current measurement, so you should call
     * this from, e.g. Command.initialize(), not in the constructor.
     */
    public VelocityReferenceControllerR3(
            LoggerFactory parent,
            VelocitySubsystemR3 subsystem,
            ControllerR3 controller,
            ReferenceR3 reference) {
        LoggerFactory log = parent.type(this);
        m_subsystem = subsystem;
        m_controller = controller;
        m_reference = reference;
        m_log_measurement = log.modelR3Logger(Level.TRACE, "measurement");
        m_log_current = log.modelR3Logger(Level.TRACE, "current");
        m_log_next = log.controlR3Logger(Level.TRACE, "next");
        m_log_error = log.modelR3Logger(Level.TRACE, "error");

        m_controller.reset();
        // initialize here so that the "done" state knows about the clock
        m_reference.initialize(m_subsystem.getState());
    }

    public void execute() {
        try {
            ModelR3 measurement = m_subsystem.getState();
            ModelR3 current = m_reference.current();
            ControlR3 next = m_reference.next();
            ModelR3 error = current.minus(measurement);
            GlobalVelocityR3 fieldRelativeTarget = m_controller.calculate(
                    measurement, current, next);
            if (DEBUG) {
                System.out.printf("ReferenceController.execute() measurement %s current %s next %s output %s\n",
                        measurement, current, next, fieldRelativeTarget);
            }

            m_subsystem.setVelocity(fieldRelativeTarget);
            m_log_measurement.log(() -> measurement);
            m_log_current.log(() -> current);
            m_log_next.log(() -> next);
            m_log_error.log(() -> error);
        } catch (IllegalStateException ex) {
            // System.out.println(ex);
            // This happens when the trajectory generator produces an empty trajectory.
            // Ignore it for now.
        }
    }

    /** Trajectory is complete and controller error is within tolerance. */
    public boolean isDone() {
        return m_reference.done() && m_controller.atReference();
    }

    public boolean atReference() {
        return m_controller.atReference();
    }

    /** Distance between the measurement and the goal. */
    public double toGo() {
        ModelR3 goal = m_reference.goal();
        ModelR3 measurement = m_subsystem.getState();
        return goal.minus(measurement).translation().getNorm();
    }
}
