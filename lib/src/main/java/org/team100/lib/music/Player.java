package org.team100.lib.music;

import java.util.List;

public interface Player {
    /** Unison */
    void play(double freq);

    default List<Player> players() {
        return List.of(this);
    }
}