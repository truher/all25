package org.team100.lib.commands.r3;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.VelocitySubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Use a profile with feedforward control only.
 * 
 * This is mainly useful for testing.
 */
public class VelocityFeedforwardOnly extends MoveAndHold {
    private static final boolean DEBUG = false;

    private final LoggerFactory m_log;
    private final HolonomicProfile m_profile;
    private final Pose2d m_goal;
    private final VelocitySubsystemR3 m_drive;

    private ProfileReferenceR3 m_reference;

    public VelocityFeedforwardOnly(
            LoggerFactory parent,
            HolonomicProfile profile,
            Pose2d goal,
            VelocitySubsystemR3 drive) {
        m_log = parent.type(this);
        m_profile = profile;
        m_goal = goal;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_reference = new ProfileReferenceR3(m_log, m_profile, "feedforward only");
        m_reference.setGoal(new ModelR3(m_goal));
        m_reference.initialize(m_drive.getState());
    }

    @Override
    public void execute() {
        GlobalVelocityR3 velocity = m_reference.next().velocity();
        if (DEBUG)
            System.out.printf("velocity %s\n", velocity);
        m_drive.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
    }

    @Override
    public boolean isDone() {
        return m_reference.done();
    }

    @Override
    public double toGo() {
        ModelR3 goal = m_reference.goal();
        ModelR3 measurement = m_drive.getState();
        return goal.minus(measurement).translation().getNorm();
    }
}
