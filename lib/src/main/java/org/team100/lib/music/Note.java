package org.team100.lib.music;

/**
 * MIDI notes from C3 to C5, equal temperament.
 * 
 * https://audiodev.blog/midi-note-chart/
 */
public enum Note {
    C3(131),
    Cs3(139),
    D3(147),
    Ds3(156),
    E3(165),
    F3(175),
    Fs3(185),
    G3(196),
    Gs3(208),
    A3(220),
    As3(233),
    B3(247),
    C4(262),
    Cs4(277),
    D4(294),
    Ds4(311),
    E4(330),
    F4(349),
    Fs4(370),
    G4(392),
    Gs4(415),
    A4(440),
    As4(466),
    B4(494),
    C5(523);

    public final int hz;

    Note(int hz) {
        this.hz = hz;
    }
}
