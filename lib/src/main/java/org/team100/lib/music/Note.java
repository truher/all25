package org.team100.lib.music;

/**
 * At the moment this matches the MiniLab3 notes, i.e. the white notes.
 * 
 * This is equal temperament.
 * 
 * TODO: add more notes.
 */
public enum Note {
    C3(131),
    D3(147),
    E3(165),
    F3(175),
    G3(196),
    A3(220),
    B3(247),
    C4(262),
    D4(294),
    E4(330),
    F4(349),
    G4(392),
    A4(440),
    B4(494),
    C5(523);

    private final int hz;

    Note(int hz) {
        this.hz = hz;
    }
}
