package org.team100.lib.subsystems.prr;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsystemPRR extends Subsystem {
    /** Position, velocity, and acceleration. May compute dynamic forces too. */
    void set(EAWConfig c, JointVelocities jv, JointAccelerations ja);

    /** Current joint positions. */
    EAWConfig getConfig();

    /** Current joint velocities. */
    JointVelocities getJointVelocity();
}
