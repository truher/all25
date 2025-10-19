package org.team100.lib.commands.r3;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Use a profile with feedforward control only.
 * 
 * TODO: remove these type parameters, make swerve stuff work for mecanum.
 */
public class FeedforwardOnly<T extends Subsystem & Consumer<GlobalVelocityR3> & Supplier<Pose2d>>
        extends MoveAndHold {
    private static final boolean DEBUG = false;
    
    private final HolonomicProfile m_profile;
    private final Pose2d m_goal;
    private final T m_drive;

    private ProfileReferenceR3 m_reference;

    public FeedforwardOnly(
            HolonomicProfile profile,
            Pose2d goal,
            T drive) {
        m_profile = profile;
        m_goal = goal;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_reference = new ProfileReferenceR3(m_profile, "feedforward only");
        m_reference.setGoal(new ModelR3(m_goal));
        m_reference.initialize(new ModelR3(m_drive.get()));
    }

    @Override
    public void execute() {
        GlobalVelocityR3 velocity = m_reference.next().velocity();
        if (DEBUG)
            System.out.printf("velocity %s\n", velocity);
        m_drive.accept(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.accept(new GlobalVelocityR3(0, 0, 0));
        m_reference.end();
    }

    @Override
    public boolean isDone() {
        return m_reference.done();
    }

    @Override
    public double toGo() {
        ModelR3 goal = m_reference.goal();
        ModelR3 measurement = new ModelR3(m_drive.get());
        return goal.minus(measurement).translation().getNorm();
    }
}
