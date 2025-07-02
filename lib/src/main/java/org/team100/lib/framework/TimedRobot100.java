package org.team100.lib.framework;

import java.util.PriorityQueue;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.IterativeRobotBase;

/**
 * Copy of {@link edu.wpi.first.wpilibj.TimedRobot} in an effort to improve
 * instrumentation.
 */
public class TimedRobot100 extends IterativeRobotBase implements Glassy {

    static class Callback implements Comparable<Callback> {
        public Runnable func;
        public double period;
        public double expirationTime;
        public DoubleLogger logger;

        /**
         * Construct a callback container.
         *
         * @param func             The callback to run.
         * @param startTimeSeconds The common starting point for all callback scheduling
         *                         in seconds.
         * @param periodSeconds    The period at which to run the callback in seconds.
         * @param offsetSeconds    The offset from the common starting time in seconds.
         * @param name             for logging
         */
        Callback(LoggerFactory logger, Runnable func, double startTimeSeconds, double periodSeconds,
                double offsetSeconds, String name) {
            this.func = func;
            this.period = periodSeconds;
            this.expirationTime = startTimeSeconds
                    + offsetSeconds
                    + Math.floor((Takt.actual() - startTimeSeconds) / this.period)
                            * this.period
                    + this.period;
            this.logger = logger.doubleLogger(Level.COMP, "duration (s)/" + name);
        }

        public void run() {

            double startWaitingS = Takt.actual();
            func.run();
            double endWaitingS = Takt.actual();
            double durationS = endWaitingS - startWaitingS;
            this.logger.log(() -> durationS);

        }

        @Override
        public boolean equals(Object rhs) {
            if (rhs instanceof Callback) {
                return Double.compare(expirationTime, ((Callback) rhs).expirationTime) == 0;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(expirationTime);
        }

        @Override
        public int compareTo(Callback rhs) {
            // Elements with sooner expiration times are sorted as lesser. The head of
            // Java's PriorityQueue is the least element.
            return Double.compare(expirationTime, rhs.expirationTime);
        }
    }

    /**
     * Fixed loop period.
     * All uses of dt should refer to TimedRobot100.LOOP_PERIOD_S.
     */
    public static final double LOOP_PERIOD_S = 0.02;

    /** An exception to the no-member rule. */
    protected final LoggerFactory m_robotLogger;

    // The C pointer to the notifier object. We don't use it directly, it is
    // just passed to the JNI bindings.
    private final int m_notifier = NotifierJNI.initializeNotifier();

    private double m_startTime;

    private final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();

    private final DoubleLogger m_log_slack;

    protected TimedRobot100() {
        super(LOOP_PERIOD_S);
        m_robotLogger = Logging.instance().rootLogger.child(this);
        m_log_slack = m_robotLogger.doubleLogger(Level.COMP, "slack time (s)");
        m_startTime = Takt.actual();
        addPeriodic(this::loopFunc, TimedRobot100.LOOP_PERIOD_S, "main loop");
        NotifierJNI.setNotifierName(m_notifier, "TimedRobot");
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(m_notifier);
        NotifierJNI.cleanNotifier(m_notifier);
    }

    /** Provide an alternate "main loop" via startCompetition(). */
    @Override
    public void startCompetition() {
        robotInit();

        if (isSimulation()) {
            simulationInit();
        }

        // Tell the DS that the robot is ready to be enabled
        Util.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            // We don't have to check there's an element in the queue first because
            // there's always at least one (the constructor adds one). It's reenqueued
            // at the end of the loop.
            Callback callback = m_callbacks.poll();

            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (callback.expirationTime * 1e6));

            // how long do we spend waiting?
            double startWaitingS = Takt.actual();
            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                // someone called StopNotifier
                break;
            }
            double endWaitingS = Takt.actual();
            double slackS = endWaitingS - startWaitingS;
            // this is the main loop slack, don't let it go to zero!
            if (Logging.instance().getLevel().admit(Level.TRACE) && slackS < 0.001) {
                Util.warnf("Slack time %f is too low!\n", slackS);
            }
            m_log_slack.log(() -> slackS);

            callback.run();

            callback.expirationTime += callback.period;
            m_callbacks.add(callback);

            // Process all other callbacks that are ready to run
            // note when we're falling behind, we stay in this inner loop,
            // perhaps never touching the outer loop.
            while ((long) (m_callbacks.peek().expirationTime * 1e6) <= curTime) {
                callback = m_callbacks.poll();

                callback.run();

                callback.expirationTime += callback.period;
                m_callbacks.add(callback);
            }
        }
    }

    /** Ends the main loop in startCompetition(). */
    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(m_notifier);
    }

    /**
     * Add a callback to run at a specific period.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     */
    public final void addPeriodic(Runnable callback, double periodSeconds, String name) {
        m_callbacks.add(new Callback(m_robotLogger, callback, m_startTime, periodSeconds, 0.0, name));
    }

    /**
     * Add a callback to run at a specific period with a starting time offset.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     * @param offsetSeconds The offset from the common starting time in seconds.
     *                      This is useful for
     *                      scheduling a callback in a different timeslot relative
     *                      to TimedRobot.
     */
    public final void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds, String name) {
        m_callbacks.add(new Callback(m_robotLogger, callback, m_startTime, periodSeconds, offsetSeconds, name));
    }

}
