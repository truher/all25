package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;

/**
 * Given a path, produces a trajectory, which includes the path and adds a
 * schedule.
 */
public class ScheduleGenerator {
    private static final boolean DEBUG = true;
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
            throw new IllegalArgumentException();
        int num_states = (int) Math.ceil(maxDistance / step + 1);
        List<Pose2dWithMotion> samples = new ArrayList<>(num_states);
        for (int i = 0; i < num_states; ++i) {
            // the dtheta and curvature come from here and are never changed.
            // the values here are just interpolated from the original values.
            Pose2dWithMotion state = path.sample(Math.min(i * step, maxDistance));
            if (DEBUG)
                System.out.printf("RESAMPLE: %d %s\n", i, state);
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
        List<ConstrainedState> constrainedStates = forwardPass(samples, start_vel);
        Pose2dWithMotion lastState = samples.get(samples.size() - 1);
        backwardsPass(lastState, end_vel, constrainedStates);
        return integrate(constrainedStates);
    }

    /**
     * Forward pass.
     * 
     * We look at pairs of consecutive states, where the start state has already
     * been velocity parameterized (though we may adjust the velocity downwards
     * during the backwards pass). We wish to find an acceleration that is
     * admissible at both the start and end state, as well as an admissible end
     * velocity. If there is no admissible end velocity or acceleration, we set the
     * end velocity to the state's maximum allowed velocity and will repair the
     * acceleration during the backward pass (by slowing down the predecessor).
     */
    private List<ConstrainedState> forwardPass(List<Pose2dWithMotion> samples, double start_vel) {
        ConstrainedState predecessor = new ConstrainedState(samples.get(0), 0);
        predecessor.setVelocityM_S(start_vel);
        predecessor.setMinAccel(-HIGH_ACCEL);
        predecessor.setMaxAccel(HIGH_ACCEL);

        // work forward through the samples
        List<ConstrainedState> constrainedStates = new ArrayList<>(samples.size());
        for (Pose2dWithMotion sample : samples) {
            double dsM = sample.distanceM(predecessor.getState());
            ConstrainedState constrainedState = new ConstrainedState(
                    sample, dsM + predecessor.getDistanceM());
            constrainedStates.add(constrainedState);
            forwardWork(predecessor, constrainedState);
            predecessor = constrainedState;
        }
        return constrainedStates;
    }

    private void forwardWork(ConstrainedState s0, ConstrainedState s1) {
        // constant-twist path length between states
        // actually this is now the double-geodesic metric (L2 for all 3 dimensions)
        // which means it is not just meters.
        double dsM = s1.getState().distanceM(s0.getState());

        // We may need to iterate to find the maximum end velocity and common
        // acceleration, since acceleration limits may be a function of velocity.
        while (true) {
            // first try the previous state accel to get the new state velocity
            double v1 = v1(s0.getVelocityM_S(), s0.getMaxAccel(), dsM);
            s1.setVelocityM_S(v1);

            // also use max accels for the new state accels
            s1.setMinAccel(-HIGH_ACCEL);
            s1.setMaxAccel(HIGH_ACCEL);

            // reduce velocity according to constraints
            s1.clampVelocity(m_constraints);

            // reduce accel according to constraints
            s1.clampAccel(m_constraints);

            // motionless
            if (Math.abs(dsM) < EPSILON) {
                return;
            }

            double accel = accel(s0.getVelocityM_S(), s1.getVelocityM_S(), dsM);
            if (accel > s1.getMaxAccel() + EPSILON) {
                // implied accel is too high because v1 is too high, perhaps because
                // a0 was too high, try again with the (lower) constrained value
                s0.setMaxAccel(s1.getMaxAccel());
                continue;
            }
            if (accel > s0.getMinAccel() + EPSILON) {
                // set the previous state accel to whatever the constrained velocity implies
                s0.setMaxAccel(accel);
            }
            return;
        }
    }

    /**
     * Backwards pass
     */
    private void backwardsPass(
            Pose2dWithMotion lastState,
            double end_velocity,
            List<ConstrainedState> constrainedStates) {
        // "successor" comes before in the backwards walk. start with the last state.
        ConstrainedState endState = constrainedStates.get(constrainedStates.size() - 1);
        ConstrainedState successor = new ConstrainedState(lastState, endState.getDistanceM());
        successor.setVelocityM_S(end_velocity);
        successor.setMinAccel(-HIGH_ACCEL);
        successor.setMaxAccel(HIGH_ACCEL);

        // work backwards through the states list
        for (int i = constrainedStates.size() - 1; i >= 0; --i) {
            ConstrainedState constrainedState = constrainedStates.get(i);
            backwardsWork(constrainedState, successor);
            successor = constrainedState;
        }
    }

    /** s0 is earlier, s1 is "successor", we're walking backwards. */
    private void backwardsWork(ConstrainedState s0, ConstrainedState s1) {
        // backwards (negative) distance from successor to initial state.
        double ds = s0.getDistanceM() - s1.getDistanceM();
        if (ds > 0) {
            // must be negative if we're walking backwards.
            throw new IllegalStateException();
        }

        while (true) {
            // s0 velocity can't be more than the accel implies
            // so this is actually an estimate for v0
            // min a is negative, ds is negative, so v0 is faster than v1
            double v0 = v1(s1.getVelocityM_S(), s1.getMinAccel(), ds);

            if (s0.getVelocityM_S() <= v0) {
                // s0 v is slower than implied v0, which means
                // that actual accel is larger than the min, so we're fine
                // No new limits to impose.
                return;
            }
            // s0 v is too fast, turn it down to obey v1 min accel.
            s0.setVelocityM_S(v0);

            s0.clampAccel(m_constraints);

            // motionless
            if (Math.abs(ds) < EPSILON) {
                return;
            }

            // implied accel using the constrained v0
            double accel = accel(s1.getVelocityM_S(), s0.getVelocityM_S(), ds);
            if (accel < s0.getMinAccel() - EPSILON) {
                // accel is too low which implies that s1 accel is too low, try again
                s1.setMinAccel(s0.getMinAccel());
                continue;
            }
            // set final accel to the implied value
            s1.setMinAccel(accel);
            return;
        }
    }

    /**
     * Integrate the constrained states forward in time to obtain the TimedStates.
     * 
     * last state accel is always zero, which might be wrong.
     */
    private static Trajectory100 integrate(List<ConstrainedState> states) throws TimingException {
        List<TimedPose> poses = new ArrayList<>(states.size());
        double time = 0.0; // time along path
        // this should be L2 distance
        double distance = 0.0; // distance along path
        double v0 = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            ConstrainedState state = states.get(i);
            final double ds = state.getDistanceM() - distance;
            final double v1 = state.getVelocityM_S();
            double dt = 0.0;
            if (i > 0) {
                double prevAccel = accel(v0, v1, ds);
                poses.get(i - 1).set_acceleration(prevAccel);
                dt = dt(v0, v1, ds, prevAccel);
            }
            time += dt;
            if (Double.isNaN(time) || Double.isInfinite(time)) {
                throw new TimingException();
            }
            poses.add(new TimedPose(state.getState(), time, v1, 0));
            v0 = v1;
            distance = state.getDistanceM();
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
        throw new TimingException();
    }

    /**
     * Return final velocity, v1, given initial velocity, v0, and acceleration over
     * distance ds.
     * 
     * v1 = sqrt(v0^2 + 2ads)
     * 
     * note a can be negative.
     * 
     * note ds can be negative, which implies backwards time
     * 
     * @param v0 initial velocity
     * @param a  acceleration
     * @param ds distance
     * @return final velocity
     */
    static double v1(double v0, double a, double ds) {
        /*
         * a = dv/dt
         * v = ds/dt
         * dt = ds/v
         * a = v dv/ds
         * a = v (v1-v0)/ds
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2ds
         * a = (v1^2 - v0^2)/2ds
         * 2*a*ds = v1^2 - v0^2
         * v1 = sqrt(v0^2 + 2*a*ds)
         */
        return Math.sqrt(v0 * v0 + 2.0 * a * ds);
    }

    /**
     * Return acceleration implied by the change in velocity (v0 to v1)
     * over the distance, ds.
     * 
     * a = (v1^2 - v0^2) / 2ds
     * 
     * note ds can be negative, which implies negative time.
     * 
     * @param v0 initial velocity
     * @param v1 final velocity
     * @param ds distance
     */
    public static double accel(double v0, double v1, double ds) {
        if (Math.abs(ds) < 1e-6) {
            // prevent division by zero
            return 0;
        }
        /*
         * a = dv/dt
         * v = ds/dt
         * dt = ds/v
         * a = v dv/ds
         * a = v (v1-v0)/ds
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2ds
         * a = (v1^2 - v0^2)/2ds
         */
        double a = (v1 * v1 - v0 * v0) / (2.0 * ds);
        double dv = v1 - v0;
        // this can be negative, which indicates that v1 precedes v0
        double dt = a / dv;
        return a;
    }

    public static class TimingException extends Exception {
    }
}