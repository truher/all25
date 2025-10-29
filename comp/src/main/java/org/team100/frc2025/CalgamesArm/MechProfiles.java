package org.team100.frc2025.CalgamesArm;

import org.team100.lib.commands.prr.FollowJointProfiles;
import org.team100.lib.motion.prr.Config;
import org.team100.lib.profile.incremental.CompleteProfile;
import org.team100.lib.profile.incremental.CurrentLimitedExponentialProfile;

public class MechProfiles {

    public static FollowJointProfiles WithCurrentLimitedExponentialProfile(
            CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CurrentLimitedExponentialProfile(2, 4, 5), // elevator
                new CurrentLimitedExponentialProfile(8, 8, 16), // arm
                new CurrentLimitedExponentialProfile(8, 8, 16)); // wrist
    }

    /**
     * Accelerate gently but decelerate firmly.
     * 
     * This is for paths that start with lots of gravity torque and end
     * without very much gravity torque, e.g. from "pick" to "home".
     */
    public static FollowJointProfiles slowFast(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 4, 6, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 8, 16, 16, 50, 50, 0.001), // arm
                new CompleteProfile(8, 4, 12, 16, 50, 50, 0.001)); // wrist
    }

    /**
     * Accelerate firmly but decelerate gently.
     * 
     * This is for paths that start without gravity torque but end with a lot of
     * gravity torque, e.g. from "home" to "pick".
     */
    
    public static FollowJointProfiles fastSlow(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 20, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algae(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(12, 8, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles gentle(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(2, 2, 2, 8, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algaeUp(CalgamesMech subsystem, Config goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(4, 4, 4, 8, 100, 100, 0.001), // arm
                new CompleteProfile(8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }
    
}
