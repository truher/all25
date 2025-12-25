package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.util.Math100;

/**
 * Given a path, produces a trajectory, which includes the path and adds a
 * schedule.
 */
public class ScheduleGenerator {
    private static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;
    /** this is the default, in order to make the constraints set the actual */
    private static final double HIGH_ACCEL = 1000;

    private final List<TimingConstraint> m_constraints;

    /** If you want a max velocity or accel constraint, use ConstantConstraint. */
    public ScheduleGenerator(List<TimingConstraint> constraints) {
        m_constraints = constraints;
    }

    /**
     * Samples the path evenly by distance, and then assign times to each sample.
     */
    public Trajectory100 timeParameterizeTrajectory(
            Path100 path,
            double step,
            double start_vel,
            double end_vel) {
        try {
            List<Pose2dWithMotion> samples = resample(path, step);
            return timeParameterizeTrajectory(samples, start_vel, end_vel);
        } catch (TimingException e) {
            e.printStackTrace();
            System.out.println("WARNING: Timing exception");
            return new Trajectory100();
        }
    }

    /**
     * Samples the path evenly by distance.
     */
    private List<Pose2dWithMotion> resample(Path100 path, double step) throws TimingException {
        double maxDistance = path.getMaxDistance();
        if (maxDistance == 0)
            throw new IllegalArgumentException("max distance must be greater than zero");
        int num_states = (int) Math.ceil(maxDistance / step) + 1;
        if (DEBUG)
            System.out.printf("resample max distance %f step %f num states %d f %f\n",
                    maxDistance, step, num_states, maxDistance / step);
        List<Pose2dWithMotion> samples = new ArrayList<>(num_states);
        for (int i = 0; i < num_states; ++i) {
            // the dtheta and curvature come from here and are never changed.
            // the values here are just interpolated from the original values.
            double d = Math.min(i * step, maxDistance);
            Pose2dWithMotion state = path.sample(d);
            if (DEBUG)
                System.out.printf("RESAMPLE: i=%d d=%f state=%s\n", i, d, state);
            samples.add(state);
        }
        return samples;
    }

    /**
     * input is some set of samples (could be evenly sampled or not), output is
     * these same samples with time.
     */
    public Trajectory100 timeParameterizeTrajectory(
            List<Pose2dWithMotion> samples,
            double start_vel,
            double end_vel) throws TimingException {
        if (samples.size() < 3)
            throw new IllegalArgumentException("must have at least three samples");
        // note below we look again at this sample. I think this exists
        // only to supply the start velocity.
        ConstrainedState s0 = new ConstrainedState(samples.get(0), 0);
        s0.velocity = start_vel;
        s0.decel = -HIGH_ACCEL;
        s0.accel = HIGH_ACCEL;

        //
        // Forward pass.
        //
        // We look at pairs of consecutive states, where the start state has already
        // been velocity parameterized (though we may adjust the velocity downwards
        // during the backwards pass). We wish to find an acceleration that is
        // admissible at both the start and end state, as well as an admissible end
        // velocity. If there is no admissible end velocity or acceleration, we set the
        // end velocity to the state's maximum allowed velocity and will repair the
        // acceleration during the backward pass (by slowing down the predecessor).
        //

        // work forward through the samples
        List<ConstrainedState> constrainedStates = new ArrayList<>(samples.size());
        for (int i = 0; i < samples.size(); ++i) {
            Pose2dWithMotion sample = samples.get(i);
            double dsM = sample.distanceCartesian(s0.state);
            if (i > 0 && i < samples.size() - 1 && dsM < 1e-6) {
                // the first distance is zero because of the weird loop structure.
                // the last distance can be zero if the step size exactly divides the path
                // length
                throw new IllegalStateException("zero distance not allowed");
            }

            ConstrainedState s1 = new ConstrainedState(sample, dsM + s0.distance);
            constrainedStates.add(s1);

            // R2 translation distance between states
            // not constant-twist arc
            // not double-geodesic with rotation
            // Just translation, so that the pathwise velocity matches
            // the curvature in the state.
            double dsM1 = s1.state.distanceCartesian(s0.state);

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // first try the previous state accel to get the new state velocity
                double v1 = Math100.v1(s0.velocity, s0.accel, dsM1);

                s1.velocity = v1;

                // also use max accels for the new state accels
                s1.decel = -HIGH_ACCEL;
                s1.accel = HIGH_ACCEL;

                // reduce velocity according to constraints
                for (TimingConstraint constraint : m_constraints) {
                    s1.velocity = Math.min(s1.velocity, constraint.maxV(s1.state));
                }

                for (TimingConstraint constraint1 : m_constraints) {
                    s1.decel = Math.max(s1.decel, constraint1.maxDecel(s1.state, s1.velocity));
                    s1.accel = Math.min(s1.accel, constraint1.maxAccel(s1.state, s1.velocity));
                }

                // motionless
                if (Math.abs(dsM1) < EPSILON) {
                    break;
                }

                double accel = Math100.accel(s0.velocity, s1.velocity, dsM1);
                // in the failure case, max accel is zero so this always fails.
                if (accel > s1.accel + EPSILON) {
                    // implied accel is too high because v1 is too high, perhaps because
                    // a0 was too high, try again with the (lower) constrained value
                    //
                    // if the constrained value is zero, this doesn't work at all.

                    s0.accel = s1.accel;
                    continue;
                }
                if (accel > s0.decel + EPSILON) {
                    // set the previous state accel to whatever the constrained velocity implies
                    s0.accel = accel;
                }
                break;
            }

            s0 = s1;
        }

  

        Pose2dWithMotion lastState = samples.get(samples.size() - 1);

        //
        // Backwards pass
        //

        // "successor" comes before in the backwards walk. start with the last state.
        ConstrainedState endState = constrainedStates.get(constrainedStates.size() - 1);
        ConstrainedState s1 = new ConstrainedState(
                lastState, endState.distance);
        s1.velocity = end_vel;
        s1.decel = -HIGH_ACCEL;
        s1.accel = HIGH_ACCEL;

        // work backwards through the states list
        for (int i = constrainedStates.size() - 1; i >= 0; --i) {
            ConstrainedState s01 = constrainedStates.get(i);

            // backwards (negative) distance from successor to initial state.
            double ds = s01.distance - s1.distance;
            if (ds > 0) {
                // must be negative if we're walking backwards.
                throw new IllegalStateException();
            }

            while (true) {
                // s0 velocity can't be more than the accel implies
                // so this is actually an estimate for v0
                // min a is negative, ds is negative, so v0 is faster than v1
                double v0 = Math100.v1(s1.velocity, s1.decel, ds);

                if (s01.velocity <= v0) {
                    // s0 v is slower than implied v0, which means
                    // that actual accel is larger than the min, so we're fine
                    // No new limits to impose.
                    break;
                }
                // s0 v is too fast, turn it down to obey v1 min accel.
                s01.velocity = v0;

                for (TimingConstraint constraint : m_constraints) {
                    s01.decel = Math.max(s01.decel, constraint.maxDecel(s01.state, s01.velocity));
                    s01.accel = Math.min(s01.accel, constraint.maxAccel(s01.state, s01.velocity));
                }

                // motionless
                if (Math.abs(ds) < EPSILON) {
                    break;
                }

                // implied accel using the constrained v0
                double accel = Math100.accel(s1.velocity, s01.velocity, ds);
                if (accel < s01.decel - EPSILON) {
                    // accel is too low which implies that s1 accel is too low, try again
                    s1.decel = s01.decel;
                    continue;
                }
                // set final accel to the implied value
                s1.decel = accel;
                break;
            }

            s1 = s01;
        }

        //
        // Integrate the constrained states forward in time to obtain the TimedStates.
        //
        // last state accel is always zero, which might be wrong.
        //

        List<TimedState> poses = new ArrayList<>(constrainedStates.size());
        double time = 0.0;
        // distance along path
        // for turn-in-place,
        double distance = 0.0;
        double v0 = 0.0;

        for (int i = 0; i < constrainedStates.size(); ++i) {
            ConstrainedState state = constrainedStates.get(i);
            double ds = state.distance - distance;
            double v1 = state.velocity;

            double dt = 0.0;
            if (i > 0) {
                double prevAccel = Math100.accel(v0, v1, ds);
                poses.get(i - 1).set_acceleration(prevAccel);
                dt = dt(v0, v1, ds, prevAccel);
            }
            time += dt;
            if (Double.isNaN(time) || Double.isInfinite(time)) {
                throw new TimingException();
            }
            poses.add(new TimedState(state.state, time, v1, 0));
            v0 = v1;
            distance = state.distance;
        }

        return new Trajectory100(poses);
    }

    /**
     * If accelerating, find the time to go from v0 to v1. Otherwise find the time
     * to go distance ds at speed v0.
     */
    private static double dt(
            double v0,
            double v1,
            double ds,
            double accel) throws TimingException {
        if (Math.abs(accel) > EPSILON) {
            return (v1 - v0) / accel;
        }
        if (Math.abs(v0) > EPSILON) {
            return ds / v0;
        }
        // not moving at all so dt is always zero
        return 0;
        // throw new TimingException(String.format("%f %f %f %f", v0, v1, ds, accel));
    }

    public static class TimingException extends Exception {
    }
}