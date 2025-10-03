package org.team100.lib.controller.drivetrain;

import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.reference.SwerveReference;
import org.team100.lib.util.Util;

/**
 * Actuates the drivetrain based on a SwerveReference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class ReferenceController {
    private static final boolean DEBUG = false;
    private final DriveSubsystemInterface m_drive;
    private final SwerveController m_controller;
    private final SwerveReference m_reference;
    private final boolean m_verbatim;

    /**
     * Initializes the reference with the current measurement, so you should call
     * this from, e.g. Command.initialize(), not in the constructor.
     */
    public ReferenceController(
            DriveSubsystemInterface drive,
            SwerveController controller,
            SwerveReference reference,
            boolean verbatim) {
        m_drive = drive;
        m_controller = controller;
        m_reference = reference;
        m_verbatim = verbatim;

        m_controller.reset();
        // initialize here so that the "done" state knows about the clock
        m_reference.initialize(m_drive.getState());
        m_drive.resetLimiter();
    }

    public void execute() {
        try {
            SwerveModel measurement = m_drive.getState();
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, m_reference.current(), m_reference.next());
            if (DEBUG) {
                Util.printf("ReferenceController.execute() measurement %s current %s next %s output %s\n",
                        measurement,
                        m_reference.current(),
                        m_reference.next(),
                        fieldRelativeTarget);
            }
            if (m_verbatim)
                m_drive.driveInFieldCoordsVerbatim(fieldRelativeTarget);
            else
                m_drive.driveInFieldCoords(fieldRelativeTarget);
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
        SwerveModel goal = m_reference.goal();
        SwerveModel measurement = m_drive.getState();
        return goal.minus(measurement).translation().getNorm();
    }
}
