package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A collection that evicts old entries.
 * 
 * Note eviction happens at *write* time, to avoid the reader needing to know
 * the timestamp, so if you stop writing to this collection, stale entries will
 * hang around. Use cleanup() to evict old entries on demand.
 */
public class TrailingHistory<T> {
    public record ValueRecord<T>(double time, T value) {
    };

    /** Entry timeout in seconds */
    private final double m_timeout;
    private final List<ValueRecord<T>> m_entries;

    /**
     * @param timeout in seconds
     */
    public TrailingHistory(double timeout) {
        m_timeout = timeout;
        m_entries = new ArrayList<ValueRecord<T>>();
    }

    /** Remove stale entries and add the new value. */
    public void add(double time, T value) {
        cleanup(time);
        m_entries.add(new ValueRecord<>(time, value));
    }

    /** Remove stale entries and add all the values. */
    public void addAll(double time, Collection<T> values) {
        cleanup(time);
        m_entries.addAll(
                values.stream()
                        .map((x) -> new ValueRecord<>(time, x))
                        .collect(Collectors.toList()));
    }

    public List<T> getAll() {
        return m_entries.stream()
                .map((x) -> x.value)
                .collect(Collectors.toUnmodifiableList());
    }

    /** Mutating iterator for filtering. */
    public Iterator<ValueRecord<T>> iterator() {
        return m_entries.iterator();
    }

    public int size() {
        return m_entries.size();
    }

    /** Remove stale entries. */
    public void cleanup(double time) {
        double horizon = time - m_timeout;
        m_entries.removeIf(x -> x.time < horizon);
    }

}
