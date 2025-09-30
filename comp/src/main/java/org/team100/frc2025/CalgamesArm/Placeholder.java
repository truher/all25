package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.select;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.motion.Config;
import org.team100.lib.trajectory.Trajectory100;

import edu.wpi.first.wpilibj2.command.Command;

/** This exists to accumulate API for the arm. */
public class Placeholder {

    // the trajecotry planner object to use for eveyrthing. This assumes same
    // constraints for all moving
    Trajectories trajectories = new Trajectories();
    // i addded some constraints. prolly something wrong - kym
    private final CalgamesMech m_CalgamesMech;

    public Placeholder(CalgamesMech cMech) {
        m_CalgamesMech = cMech;
    }

    /**
     * Move to central lower stowed position and hold it there forever.
     * This should also be the default command.
     */
    public Command stow() {
        return print("stow");
    }

    /**  */
    public boolean stowed() {
        return false;
    }

    /** True if the center of gravity is not too high. */
    public boolean isSafeToDrive() {
        return false;
    }

    /** Flop the arm into the station and hold it there forever. */
    public FollowTrajectory station() {
        // NOTE: be aware of position, don't try to do this if too close to the reef or
        // the edge of the field.
        // TODO: Add logic for not deploying to close to stuff
        Trajectory100 station = trajectories.betweenToStation();
        return new FollowTrajectory(m_CalgamesMech, station);
    }

    public FollowTrajectory AlgaeGroundPick() {
        // NOTE: be aware of position, don't try to do this if too close to the reef or
        // the edge of the field.
        // TODO: Add logic for not deploying to close to stuff
        Trajectory100 apick = trajectories.betweenToAPICK();
        return new FollowTrajectory(m_CalgamesMech, apick);
    }

    /** Move the arm to the coral floor pick and hold there forever. */
    public FollowTrajectory pick() {
        // NOTE: be aware of position, don't try to do this if too close to the reef or
        // the edge of the field.
        // TODO: Add logic for not deploying to close to stuff
        Trajectory100 pick = trajectories.betweenToPick();
        return new FollowTrajectory(m_CalgamesMech, pick);
    }

    /** Use a profile to move from anywhere to the floor, and stay there forever. */
    public Command pickWithProfile() {
        Config goal = new Config(0, -3 * Math.PI / 4, Math.PI / 4);
        return new FollowJointProfiles(m_CalgamesMech, goal);
    }

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        Trajectory100 A1 = trajectories.betweenToA1(); // the higher algae
        Trajectory100 A2 = trajectories.betweenToA2();

        return select(
                Map.ofEntries(
                        Map.entry(
                                ScoringLevel.L3,
                                new FollowTrajectory(m_CalgamesMech, A1)),
                        Map.entry(
                                ScoringLevel.L2,
                                new FollowTrajectory(m_CalgamesMech, A2))),
                level);
    }

    /**
     * Move to the L1 scoring position and hold there forever
     */
    public FollowTrajectory prePlaceL1() {
        Trajectory100 ppL1 = trajectories.betweenToL1();
        return new FollowTrajectory(m_CalgamesMech, ppL1);
    }

    /**
     * Move to the L2 scoring position and hold there forever
     */
    public FollowTrajectory prePlaceL2() {
        Trajectory100 ppL2 = trajectories.betweenToL2();
        return new FollowTrajectory(m_CalgamesMech, ppL2);
    }

    /**
     * True if the L2 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    public boolean atL2() {
        return false;
    }

    /**
     * Move to the L3 scoring position and hold there forever
     */
    public FollowTrajectory prePlaceL3() {
        Trajectory100 ppL3 = trajectories.betweenToL3();
        return new FollowTrajectory(m_CalgamesMech, ppL3);
    }

    /**
     * True if the L3 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    public boolean atL3() {
        return false;
    }

    /**
     * Move to the L4 scoring position and hold there forever
     */
    public FollowTrajectory prePlaceL4() {
        Trajectory100 ppL4 = trajectories.betweenToL4();
        return new FollowTrajectory(m_CalgamesMech, ppL4); // TODO: Check this code kai made it
    }

    /**
     * Move to the barge scoring position and hold there forever
     */
    public FollowTrajectory prePlaceBarge() {
        Trajectory100 ppBarge = trajectories.betweenToBarge();
        return new FollowTrajectory(m_CalgamesMech, ppBarge); // TODO: Check this code kai made it
    }

    /**
     * Move to the proccessor scoring position and hold there forever
     */
    public FollowTrajectory prePlaceProcessor() {
        Trajectory100 ppProcessor = trajectories.betweenToProcessor();
        return new FollowTrajectory(m_CalgamesMech, ppProcessor); // TODO: Check this code kai made it
    }

    /**
     * True if the L3 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    // 9/27 kai - this whole atL4 thing should be useless now, we are using the
    // follow trajecotry object
    public boolean atL4() {
        return false;
    }

}
