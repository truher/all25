package org.team100.lib.commands.r3.helper;

import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR3Logger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.SubsystemR3;

public abstract class ReferenceControllerR3Base {
    private final SubsystemR3 m_subsystem;
    private final ControllerR3 m_controller;
    private final ReferenceR3 m_reference;
    private final ModelR3Logger m_log_measurement;
    private final ModelR3Logger m_log_current;
    private final ControlR3Logger m_log_next;
    private final ModelR3Logger m_log_error;

    /**
     * Initializes the reference with the current measurement.
     */
    ReferenceControllerR3Base(
            LoggerFactory parent,
            SubsystemR3 subsystem,
            ControllerR3 controller,
            ReferenceR3 reference) {
        m_subsystem = subsystem;
        m_controller = controller;
        m_reference = reference;
        LoggerFactory log = parent.type(this);
        m_log_measurement = log.modelR3Logger(Level.TRACE, "measurement");
        m_log_current = log.modelR3Logger(Level.TRACE, "current");
        m_log_next = log.controlR3Logger(Level.TRACE, "next");
        m_log_error = log.modelR3Logger(Level.TRACE, "error");
        // initialize here so that the "done" state knows about the clock
        m_reference.initialize(subsystem.getState());
    }

    /**
     * Actuate the subsystem here.
     * 
     * @param next The next control setpoint.
     * @param u    The controller output.
     */
    abstract void execute100(ControlR3 next, GlobalVelocityR3 u);

    /**
     * This should be called in Command.execute().
     */
    public void execute() {
        try {
            ModelR3 measurement = m_subsystem.getState();
            ModelR3 current = m_reference.current();
            ControlR3 next = m_reference.next();
            ModelR3 error = current.minus(measurement);
            GlobalVelocityR3 u = m_controller.calculate(measurement, current, next);
            execute100(next, u);
            m_log_measurement.log(() -> measurement);
            m_log_current.log(() -> current);
            m_log_next.log(() -> next);
            m_log_error.log(() -> error);
        } catch (IllegalStateException ex) {
            // This happens when the trajectory generator produces an empty trajectory.
        }
    }

    /**
     * Trajectory is complete, and controller error is within tolerance.
     * 
     * Use this with "until()" to end the command.
     */
    public boolean isDone() {
        return m_reference.done() && m_controller.atReference();
    }

    /**
     * Distance between the measurement and the goal.
     * 
     * Use this to start, or finish, parallel commands while this one is still
     * running, e.g. "start doing X if Y is near the goal."
     */
    public double toGo() {
        ModelR3 goal = m_reference.goal();
        ModelR3 measurement = m_subsystem.getState();
        return goal.minus(measurement).translation().getNorm();
    }

}
