package org.team100.lib.field;

import org.junit.jupiter.api.Test;
import org.team100.lib.field.FieldConstants.ReefPoint;

public class SetpointsTest {
    @Test
    void testSetA() {
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.A, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.AB, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.B, 1.4));

        System.out.println(FieldConstants.getScoringDestination(ReefPoint.C, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.CD, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.D, 1.4));

        System.out.println(FieldConstants.getScoringDestination(ReefPoint.E, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.EF, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.F, 1.4));

        System.out.println(FieldConstants.getScoringDestination(ReefPoint.G, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.GH, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.H, 1.4));

        System.out.println(FieldConstants.getScoringDestination(ReefPoint.I, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.IJ, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.J, 1.4));

        System.out.println(FieldConstants.getScoringDestination(ReefPoint.K, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.KL, 1.4));
        System.out.println(FieldConstants.getScoringDestination(ReefPoint.L, 1.4));
    }
}
