package org.team100.lib.coherence;

import java.util.function.Supplier;

/** Cache an object supplier */
public class ObjectCache<T> implements Supplier<T> {
    private final Supplier<T> m_delegate;
    private T m_value;

    /** Do not call this directly, use Cache.of(). */
    ObjectCache(Supplier<T> delegate) {
        m_delegate = delegate;
        m_value = null;
    }

    /**
     * Use the cached value if it exists, otherwise ask the delegate, cache, and
     * return the value.
     */
    @Override
    public synchronized T get() {
        // synchronized adds ~20ns.
        if (m_value == null)
            m_value = m_delegate.get();

        return m_value;
    }

    /**
     * Erase the cache so the next get() will ask the delegate. You should generally
     * let robotPeriodic() do this, but it's ok to force it, e.g. in resetPose().
     */
    public synchronized void reset() {
        m_value = null;
    }

    /**
     * Force the cache to contain the value, effectively overriding the delegate's
     * previous output.
     */
    public synchronized void set(T value) {
        m_value = value;
    }

    /**
     * Stop updating this cache.
     *
     * There's no way to restart the updates, so you
     * should discard this object once you call end().
     */
    public void end() {
        Cache.removeObjectCache(this);
    }
}