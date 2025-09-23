package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Collection;
<<<<<<< Updated upstream
import java.util.Iterator;
=======
>>>>>>> Stashed changes
import java.util.List;
import java.util.stream.Collectors;

/**
 * A collection that evicts old entries.
 */
public class TrailingHistory<T> {
<<<<<<< Updated upstream
    public record ValueRecord<T>(double time, T value) {
=======
    private record ValueRecord<T>(double time, T value) {
>>>>>>> Stashed changes
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

    /** Remove stale entries and all the values. */
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

<<<<<<< Updated upstream
    /** Mutating iterator for filtering. */
    public Iterator<ValueRecord<T>> iterator() {
        return m_entries.iterator();
    }

    public int size() {
        return m_entries.size();
    }

=======
>>>>>>> Stashed changes
    //////////////////

    private void cleanup(double time) {
        double horizon = time - m_timeout;
        m_entries.removeIf(x -> x.time < horizon);
    }

}
