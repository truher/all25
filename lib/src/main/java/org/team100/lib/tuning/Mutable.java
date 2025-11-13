package org.team100.lib.tuning;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A mutable double linked to Network Tables, for tuning.
 * 
 * Use the DoubleSupplier API for polling.
 * Provide a DoubleConsumer to get called on changes.
 * Values do not survive restarts.
 */
public class Mutable implements DoubleSupplier {
    private static final boolean FATAL = false;
    private static final Map<String, DoubleEntry> ALL_ENTRIES = new HashMap<>();
    private final DoubleEntry m_entry;
    private final DoubleConsumer m_onChange;
    private final DoubleCache m_cache;

    public Mutable(LoggerFactory log, String leaf, double defaultValue, DoubleConsumer onChange) {
        if (onChange == null)
            throw new IllegalArgumentException();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();
        String name = log.root(leaf);
        m_entry = getEntry(inst, name, defaultValue);
        m_onChange = onChange;
        m_cache = Cache.ofDouble(this::update);
    }

    private static DoubleEntry getEntry(NetworkTableInstance inst, String name, double defaultValue) {
        if (ALL_ENTRIES.containsKey(name)) {
            if (FATAL)
                throw new IllegalArgumentException(name);
            System.out.printf("** WARNING: duplicate Mutable name %s\n", name);
            return ALL_ENTRIES.get(name);
        }
        DoubleTopic topic = inst.getDoubleTopic(name);
        DoubleEntry entry = topic.getEntry(defaultValue);
        // Makes sure we don't get a stale value.
        entry.set(defaultValue);
        // You can't use "persistent" here, because then the key goes in the RoboRIO
        // networktables.json file and can never be deleted (except manually).
        topic.setRetained(true);
        // make sure the queue is empty
        entry.readQueue();
        ALL_ENTRIES.put(name, entry);
        return entry;
    }

    /** Use this in tests to avoid mixing values */
    public static void unpublishAll() {
        for (DoubleEntry e : ALL_ENTRIES.values()) {
            e.unpublish();
        }
        ALL_ENTRIES.clear();
    }

    /** if you don't care to subscribe to changes */
    public Mutable(LoggerFactory log, String leaf, double defaultValue) {
        this(log, leaf, defaultValue, (x) -> {
        });
    }

    /** Supply the current value. */
    @Override
    public double getAsDouble() {
        return m_cache.getAsDouble();
    }

    /** Updates the cache and also notifies the consumer. */
    private double update() {
        double[] queue = m_entry.readQueueValues();
        double val = m_entry.get();
        if (queue.length > 0)
            m_onChange.accept(val);
        return val;
    }

}
