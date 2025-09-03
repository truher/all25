package org.team100.frc2025;

import org.junit.jupiter.api.Test;
import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;

import edu.wpi.first.math.geometry.Translation2d;

public class SetpointsTest {
    @Test
    void testSetA(){
        Translation2d t = FieldConstants.getScoringDestination(FieldSector.AB, ReefDestination.LEFT, 1.4);
        Translation2d t1 = FieldConstants.getScoringDestination(FieldSector.AB, ReefDestination.RIGHT, 1.4);
        Translation2d t2 = FieldConstants.getScoringDestination(FieldSector.CD, ReefDestination.LEFT, 1.4);
        Translation2d t3 = FieldConstants.getScoringDestination(FieldSector.CD, ReefDestination.RIGHT, 1.4);
        Translation2d t4 = FieldConstants.getScoringDestination(FieldSector.EF, ReefDestination.LEFT, 1.4);
        Translation2d t5 = FieldConstants.getScoringDestination(FieldSector.EF, ReefDestination.RIGHT, 1.4);
        Translation2d t6 = FieldConstants.getScoringDestination(FieldSector.GH, ReefDestination.LEFT, 1.4);
        Translation2d t7 = FieldConstants.getScoringDestination(FieldSector.GH, ReefDestination.RIGHT, 1.4);
        Translation2d t8 = FieldConstants.getScoringDestination(FieldSector.IJ, ReefDestination.LEFT, 1.4);
        Translation2d t9 = FieldConstants.getScoringDestination(FieldSector.IJ, ReefDestination.RIGHT, 1.4);
        Translation2d t10 = FieldConstants.getScoringDestination(FieldSector.KL, ReefDestination.LEFT, 1.4);
        Translation2d t11 = FieldConstants.getScoringDestination(FieldSector.KL, ReefDestination.RIGHT, 1.4);
        System.out.println(t);
        System.out.println(t1);
        System.out.println(t2);
        System.out.println(t3);
        System.out.println(t4);
        System.out.println(t5);
        System.out.println(t6);
        System.out.println(t7);
        System.out.println(t8);
        System.out.println(t9);
        System.out.println(t10);
        System.out.println(t11);

    }

}
