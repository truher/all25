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
    public static class TimingException extends Exception {
    }

    public static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;
    /** this is the default, in order to make the constraints set the actual */
    private static final double HIGH_ACCEL = 1000;

    private final List<TimingConstraint> m_constraints;

    public ScheduleGenerator(List<TimingConstraint> constraints) {
        m_constraints = constraints;
    }

    /**
     * Samples the path evenly by distance, then assigns a time to each sample.
     */
    public Trajectory100 timeParameterizeTrajectory(
            Path100 path,
            double step,
            double start_vel,
            double end_vel) {
        try {
            Pose2dWithMotion[] samples = path.resample(step);
            return timeParameterizeTrajectory(samples, start_vel, end_vel);
        } catch (TimingException e) {
            e.printStackTrace();
            System.out.println("WARNING: Timing exception");
            return new Trajectory100();
        }
    }

    /**
     * input is some set of samples (could be evenly sampled or not), output is
     * these same samples with time.
     */
    public Trajectory100 timeParameterizeTrajectory(
            Pose2dWithMotion[] samples,
            double start_vel,
            double end_vel) throws TimingException {
        int n = samples.length;
        if (n < 3)
            throw new IllegalArgumentException("must have at least three samples");

        double distances[] = new double[n];

        // note below we look again at this sample. I think this exists
        // only to supply the start velocity.
        Pose2dWithMotion previousPose = samples[0];
        ConstrainedState s0 = new ConstrainedState();
        distances[0] = 0;
        double previousDistance = 0;
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
        ConstrainedState[] constrainedStates = new ConstrainedState[n];
        {
            for (int i = 0; i < n; ++i) {
                double arclength = samples[i].distanceCartesian(previousPose);
                if (i > 0 && i < n - 1 && arclength < 1e-6) {
                    // the first distance is zero because of the weird loop structure.
                    // the last distance can be zero if the step size exactly divides the path
                    // length
                    throw new IllegalStateException("zero distance not allowed");
                }

                distances[i] = arclength + previousDistance;

                constrainedStates[i] = new ConstrainedState();

                // R2 translation distance between states
                // not constant-twist arc
                // not double-geodesic with rotation
                // Just translation, so that the pathwise velocity matches
                // the curvature in the state.
                double arclength1 = samples[i].distanceCartesian(previousPose);

                // We may need to iterate to find the maximum end velocity and common
                // acceleration, since acceleration limits may be a function of velocity.
                while (true) {
                    // first try the previous state accel to get the new state velocity
                    double v1 = Math100.v1(s0.velocity, s0.accel, arclength1);

                    constrainedStates[i].velocity = v1;

                    // also use max accels for the new state accels
                    constrainedStates[i].decel = -HIGH_ACCEL;
                    constrainedStates[i].accel = HIGH_ACCEL;

                    // reduce velocity according to constraints
                    for (TimingConstraint constraint : m_constraints) {
                        constrainedStates[i].velocity = Math.min(constrainedStates[i].velocity,
                                constraint.maxV(samples[i]));
                    }

                    for (TimingConstraint constraint1 : m_constraints) {
                        constrainedStates[i].decel = Math.max(constrainedStates[i].decel,
                                constraint1.maxDecel(samples[i], constrainedStates[i].velocity));
                        constrainedStates[i].accel = Math.min(constrainedStates[i].accel,
                                constraint1.maxAccel(samples[i], constrainedStates[i].velocity));
                    }

                    // motionless, which can happen at the end
                    if (Math.abs(arclength1) < EPSILON) {
                        break;
                    }

                    double accel = Math100.accel(s0.velocity, constrainedStates[i].velocity, arclength1);
                    // in the failure case, max accel is zero so this always fails.
                    if (accel > constrainedStates[i].accel + EPSILON) {
                        // implied accel is too high because v1 is too high, perhaps because
                        // a0 was too high, try again with the (lower) constrained value
                        //
                        // if the constrained value is zero, this doesn't work at all.

                        s0.accel = constrainedStates[i].accel;
                        continue;
                    }
                    if (accel > s0.decel + EPSILON) {
                        // set the previous state accel to whatever the constrained velocity implies
                        s0.accel = accel;
                    }
                    break;
                }

                s0 = constrainedStates[i];
                previousPose = samples[i];
                previousDistance = distances[i];
            }
        }

        //
        // Backwards pass
        //

        {
            // "successor" comes before in the backwards walk. start with the last state.
            double successorDistance = distances[n - 1];
            ConstrainedState s1 = new ConstrainedState();
            s1.velocity = end_vel;
            s1.decel = -HIGH_ACCEL;
            s1.accel = HIGH_ACCEL;

            // work backwards through the states list
            for (int i = n - 1; i >= 0; --i) {
                // backwards (negative) distance from successor to initial state.
                double dq = distances[i] - successorDistance;
                if (dq > 0) {
                    // must be negative if we're walking backwards.
                    throw new IllegalStateException();
                }

                while (true) {
                    // s0 velocity can't be more than the accel implies
                    // so this is actually an estimate for v0
                    // min a is negative, dq is negative, so v0 is faster than v1
                    double v0 = Math100.v1(s1.velocity, s1.decel, dq);

                    if (constrainedStates[i].velocity <= v0) {
                        // s0 v is slower than implied v0, which means
                        // that actual accel is larger than the min, so we're fine
                        // No new limits to impose.
                        break;
                    }
                    // s0 v is too fast, turn it down to obey v1 min accel.
                    constrainedStates[i].velocity = v0;

                    for (TimingConstraint constraint : m_constraints) {
                        constrainedStates[i].decel = Math.max(constrainedStates[i].decel,
                                constraint.maxDecel(samples[i], constrainedStates[i].velocity));
                        constrainedStates[i].accel = Math.min(constrainedStates[i].accel,
                                constraint.maxAccel(samples[i], constrainedStates[i].velocity));
                    }

                    // motionless, which can happen at the end
                    if (Math.abs(dq) < EPSILON) {
                        break;
                    }

                    // implied accel using the constrained v0
                    double accel = Math100.accel(s1.velocity, constrainedStates[i].velocity, dq);
                    if (accel < constrainedStates[i].decel - EPSILON) {
                        // accel is too low which implies that s1 accel is too low, try again
                        s1.decel = constrainedStates[i].decel;
                        continue;
                    }
                    // set final accel to the implied value
                    s1.decel = accel;
                    break;
                }

                s1 = constrainedStates[i];
                successorDistance = distances[i];
            }
        }

        //
        // Integrate the constrained states forward in time to obtain the TimedStates.
        //
        // last state accel is always zero, which might be wrong.
        //

        List<TimedState> poses = new ArrayList<>(n);
        {
            double time = 0.0;
            double distance = 0.0;
            double v0 = 0.0;

            for (int i = 0; i < n; ++i) {
                double dq = distances[i] - distance;
                double v1 = constrainedStates[i].velocity;

                double dt = 0.0;
                if (i > 0) {
                    double prevAccel = Math100.accel(v0, v1, dq);
                    poses.get(i - 1).set_acceleration(prevAccel);
                    dt = dt(v0, v1, dq, prevAccel);
                }
                time += dt;
                if (Double.isNaN(time) || Double.isInfinite(time)) {
                    throw new TimingException();
                }
                poses.add(new TimedState(samples[i], time, v1, 0));
                v0 = v1;
                distance = distances[i];
            }
        }

        return new Trajectory100(poses);
    }

    /**
     * If accelerating, find the time to go from v0 to v1. Otherwise find the time
     * to go distance dq at speed v0.
     */
    private static double dt(
            double v0,
            double v1,
            double dq,
            double accel) throws TimingException {
        if (Math.abs(accel) > EPSILON) {
            return (v1 - v0) / accel;
        }
        if (Math.abs(v0) > EPSILON) {
            return dq / v0;
        }
        // not moving at all so dt is always zero
        return 0;
        // throw new TimingException(String.format("%f %f %f %f", v0, v1, ds, accel));
    }
}