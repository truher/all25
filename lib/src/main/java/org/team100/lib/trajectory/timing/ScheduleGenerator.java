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

    public static final boolean DEBUG = true;
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

        // distance monotonically increases
        double distances[] = new double[n];
        double velocities[] = new double[n];
        double decels[] = new double[n];
        double accels[] = new double[n];

        // Forward pass.
        {
            Pose2dWithMotion previousPose = samples[0];
            double previousDistance = 0;
            double previousVelocity = start_vel;
            double previousDecel = -HIGH_ACCEL;
            double previousAccel = HIGH_ACCEL;
            for (int i = 0; i < n; ++i) {
                // arclength is never negative but can be zero.
                double arclength = samples[i].distanceCartesian(previousPose);
                if (i > 0 && i < n - 1 && arclength < 1e-6) {
                    // the first distance is zero because of the weird loop structure.
                    // the last distance can be zero if the step size exactly divides the path
                    // length
                    throw new IllegalStateException("zero distance not allowed");
                }

                distances[i] = arclength + previousDistance;
                velocities[i] = 100;
                decels[i] = -HIGH_ACCEL;
                accels[i] = HIGH_ACCEL;

                // We may need to iterate to find the maximum end velocity and common
                // acceleration, since acceleration limits may be a function of velocity.
                while (true) {
                    // first try the previous state accel to get the new state velocity
                    double v1 = Math100.v1(previousVelocity, previousAccel, arclength);

                    velocities[i] = v1;

                    // also use max accels for the new state accels
                    // decels[i] = -HIGH_ACCEL;
                    // accels[i] = HIGH_ACCEL;

                    // reduce velocity according to constraints
                    for (TimingConstraint constraint : m_constraints) {
                        double constraintV = constraint.maxV(samples[i]);
                        if (DEBUG) {
                            System.out.printf("i %d constraint %s v %f\n",
                                    i, constraint.getClass().getSimpleName(), constraintV);
                        }
                        velocities[i] = Math.min(velocities[i], constraintV);
                    }

                    for (TimingConstraint constraint1 : m_constraints) {
                        // removing this breaks TrajectoryPlannerTest.test2d.
                        // which is an s-shaped thing with variable curvature.
                        // the passing test is definitely wrong, and the failing
                        // test is wrong too.
                        // decels[i] = Math.max(decels[i], constraint1.maxDecel(samples[i],
                        // velocities[i]));
                        accels[i] = Math.min(accels[i], constraint1.maxAccel(samples[i], velocities[i]));
                    }

                    // motionless, which can happen at the end
                    if (Math.abs(arclength) < EPSILON) {
                        break;
                    }

                    double accel = Math100.accel(previousVelocity, velocities[i], arclength);
                    // in the failure case, max accel is zero so this always fails.
                    if (accel > accels[i] + EPSILON) {
                        // implied accel is too high because v1 is too high, perhaps because
                        // a0 was too high, try again with the (lower) constrained value
                        //
                        // if the constrained value is zero, this doesn't work at all.

                        previousAccel = accels[i];
                        continue;
                    }
                    if (accel > previousDecel + EPSILON) {
                        // set the previous state accel to whatever the constrained velocity implies
                        previousAccel = accel;
                    }
                    break;
                }

                previousPose = samples[i];
                previousDistance = distances[i];
                previousVelocity = velocities[i];
                previousDecel = decels[i];
                previousAccel = accels[i];
            }
        }

        //
        // Backwards pass
        //
        {
            // "successor" comes before in the backwards walk. start with the last state.
            double successorDistance = distances[n - 1];
            double successorVelocity = end_vel;
            double successorDecel = -HIGH_ACCEL;
            double successorAccel = HIGH_ACCEL;

            // work backwards through the states list
            for (int i = n - 1; i >= 0; --i) {
                // backwards (negative) distance from successor to initial state.
                double dq = distances[i] - successorDistance;

                while (true) {
                    // s0 velocity can't be more than the accel implies
                    // so this is actually an estimate for v0
                    // min a is negative, dq is negative, so v0 is faster than v1
                    double v0 = Math100.v1(successorVelocity, successorDecel, dq);
                    if (DEBUG)
                        System.out.printf("i %d v0 %f\n", i, v0);

                    if (velocities[i] <= v0) {
                        // s0 v is slower than implied v0, which means
                        // that actual accel is larger than the min, so we're fine
                        // No new limits to impose.
                        break;
                    }
                    // v is too fast
                    if (DEBUG)
                        System.out.printf("v too fast i %d v_i %f v0 %f\n", i, velocities[i], v0);
                    velocities[i] = v0;

                    for (TimingConstraint constraint : m_constraints) {
                        decels[i] = Math.max(decels[i], constraint.maxDecel(samples[i], velocities[i]));
                        accels[i] = Math.min(accels[i], constraint.maxAccel(samples[i], velocities[i]));
                    }

                    // motionless, which can happen at the end
                    if (Math.abs(dq) < EPSILON) {
                        break;
                    }

                    // implied accel using the constrained v0
                    double accel = Math100.accel(successorVelocity, velocities[i], dq);
                    if (accel < decels[i] - EPSILON) {
                        // accel is too low which implies that s1 accel is too low, try again
                        successorDecel = decels[i];
                        continue;
                    }
                    // set final accel to the implied value
                    successorDecel = accel;
                    break;
                }

                successorDistance = distances[i];
                successorVelocity = velocities[i];
                successorDecel = decels[i];
                successorAccel = accels[i];
            }
        }

        //
        // Integrate the constrained states forward in time to obtain the TimedStates.
        //
        // last state accel is always zero, which might be wrong.
        //
        List<TimedState> poses = new ArrayList<>(n);
        {
            double runningTotalTime = 0.0;
            double previousDistance = 0.0;
            double previousVelocity = 0.0;

            for (int i = 0; i < n; ++i) {
                double dq = distances[i] - previousDistance;
                double dt = 0.0;
                if (i > 0) {
                    double prevAccel = Math100.accel(previousVelocity, velocities[i], dq);
                    poses.get(i - 1).set_acceleration(prevAccel);
                    dt = dt(previousVelocity, velocities[i], dq, prevAccel);
                }
                runningTotalTime += dt;
                if (Double.isNaN(runningTotalTime) || Double.isInfinite(runningTotalTime)) {
                    throw new TimingException();
                }
                poses.add(new TimedState(samples[i], runningTotalTime, velocities[i], 0));
                previousVelocity = velocities[i];
                previousDistance = distances[i];
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