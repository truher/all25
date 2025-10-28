package org.team100.lib.coherence;

import java.util.function.DoubleSupplier;

public class DoubleCache implements DoubleSupplier {
    private final DoubleSupplier m_delegate;
    private double m_value;
    private boolean m_valid;

    /** Do not call this directly, use Cache.ofDouble() */
    DoubleCache(DoubleSupplier delegate) {
        m_delegate = delegate;
        m_valid = false;
    }

    /**
     * Use the cached value if it exists, otherwise ask the delegate, cache, and
     * return the value.
     */
    @Override
    public synchronized double getAsDouble() {
        // synchronized adds ~20ns.
        if (m_valid)
            return m_value;
        m_value = m_delegate.getAsDouble();
        m_valid = true;
        return m_value;
    }

    /** Erase the cache so the next get() will ask the delegate. */
    public synchronized void reset() {
        m_valid = false;
    }
}