package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Field {
    //TODO double check all values in at some point, good chance something could be wrong here, rookie task?
    //^  X is left and right from origin, Y is top to bottom

    // Blue bottom corner is the origin
    public static final Pose2d origin = new Pose2d();

    // Field
    public static final Distance fieldLength = Inches.of(690.875);
    public static final Distance fieldWidth = Inches.of(317);

    // Reef
    // Cycled clockwise A - K - I - G - E - C 
    //                  B - L - J - H - F - D 
    public static final Translation2d blueReefCenter = new Translation2d(Inches.of(176.2768), Inches.of(158.4991));

    public static final Pose2d blueReefB = new Pose2d(new Translation2d(Inches.of(143.531), Inches.of(152.03)), Rotation2d.fromDegrees(180));
    public static final Pose2d blueReefL = new Pose2d(blueReefB.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));
    public static final Pose2d blueReefJ = new Pose2d(blueReefL.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(60));
    public static final Pose2d blueReefH = new Pose2d(blueReefJ.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(0));
    public static final Pose2d blueReefF = new Pose2d(blueReefH.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60));
    public static final Pose2d blueReefD = new Pose2d(blueReefF.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));

    public static final Pose2d blueReefA = new Pose2d(Inches.of(143.531), Inches.of(164.95), Rotation2d.fromDegrees(180));
    public static final Pose2d blueReefK = new Pose2d(blueReefA.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));
    public static final Pose2d blueReefI = new Pose2d(blueReefK.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(60));
    public static final Pose2d blueReefG = new Pose2d(blueReefI.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(0));
    public static final Pose2d blueReefE = new Pose2d(blueReefG.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60));
    public static final Pose2d blueReefC = new Pose2d(blueReefE.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));


    public static final Pose2d redReefA = new Pose2d(fieldLength.minus(blueReefA.getMeasureX()), blueReefA.getMeasureY(), Rotation2d.fromDegrees(0));
    public static final Pose2d redReefK = new Pose2d(fieldLength.minus(blueReefK.getMeasureX()), blueReefK.getMeasureY(), Rotation2d.fromDegrees(60));
    public static final Pose2d redReefI = new Pose2d(fieldLength.minus(blueReefI.getMeasureX()), blueReefI.getMeasureY(), Rotation2d.fromDegrees(120));
    public static final Pose2d redReefG = new Pose2d(fieldLength.minus(blueReefG.getMeasureX()), blueReefG.getMeasureY(), Rotation2d.fromDegrees(180));
    public static final Pose2d redReefE = new Pose2d(fieldLength.minus(blueReefE.getMeasureX()), blueReefE.getMeasureY(), Rotation2d.fromDegrees(-60));
    public static final Pose2d redReefC = new Pose2d(fieldLength.minus(blueReefC.getMeasureX()), blueReefC.getMeasureY(), Rotation2d.fromDegrees(-120));

    public static final Pose2d redReefB = new Pose2d(fieldLength.minus(blueReefB.getMeasureX()), blueReefB.getMeasureY(), Rotation2d.fromDegrees(0));
    public static final Pose2d redReefL = new Pose2d(fieldLength.minus(blueReefL.getMeasureX()), blueReefL.getMeasureY(), Rotation2d.fromDegrees(60));
    public static final Pose2d redReefJ = new Pose2d(fieldLength.minus(blueReefJ.getMeasureX()), blueReefJ.getMeasureY(), Rotation2d.fromDegrees(120));
    public static final Pose2d redReefH = new Pose2d(fieldLength.minus(blueReefH.getMeasureX()), blueReefH.getMeasureY(), Rotation2d.fromDegrees(180));
    public static final Pose2d redReefF = new Pose2d(fieldLength.minus(blueReefF.getMeasureX()), blueReefF.getMeasureY(), Rotation2d.fromDegrees(-60));
    public static final Pose2d redReefD = new Pose2d(fieldLength.minus(blueReefD.getMeasureX()), blueReefD.getMeasureY(), Rotation2d.fromDegrees(-120));
    
    // Reef Coral
    public static final Distance levelOneHeight = Inches.of(18);
    public static final Distance levelTwoHeight = Inches.of(31.875);
    public static final Distance levelThreeHeight = Inches.of(47.625);
    public static final Distance levelFourHeight = Inches.of(72);

    // Reef Algae
    public static final Distance algaeLowHeight = levelTwoHeight.minus(Inches.of(6.25));
    public static final Distance algaeHighHeight = levelThreeHeight.minus(Inches.of(6.25));
    
    // Barge
    public static final Translation2d blueCageTop = new Translation2d(Inches.of(345.4375), Inches.of(285.875));
    public static final Translation2d blueCageMid = new Translation2d(Inches.of(345.4375), Inches.of(242.875));
    public static final Translation2d blueCageLow = new Translation2d(Inches.of(345.4375), Inches.of(200));

    public static final Translation2d redCageTop = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageTop.getMeasureY()));
    public static final Translation2d redCageMid = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageMid.getMeasureY()));
    public static final Translation2d redCageLow = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageLow.getMeasureY()));

    // Coral Station
    public static final Translation2d blueStationLow = new Translation2d(Inches.of(130.71), Inches.of(81.69));
    public static final Translation2d blueStationTop = new Translation2d(blueStationLow.getMeasureX(), fieldWidth.minus(blueStationLow.getMeasureY()));

    public static final Translation2d redStationLow = new Translation2d(fieldLength.minus(blueStationLow.getMeasureX()), blueStationLow.getMeasureY());
    public static final Translation2d redStationTop = new Translation2d(fieldLength.minus(blueStationLow.getMeasureX()), fieldWidth.minus(blueStationLow.getMeasureY()));

    // Processor
    //^ processor location is on the edge of the arena carpet rather then the exact middle of the structure
    public static final Translation2d blueProcessor = new Translation2d(Inches.of(0), Inches.of(235.7255));
    public static final Translation2d redProcessor = new Translation2d(fieldWidth, fieldLength.minus(blueProcessor.getMeasureY()));
    
}
