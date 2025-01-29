package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.SwerveK;

public class Field {
    // TODO: double check all values in at some point, good chance something could be wrong here, rookie task?
    //^  X is left and right from origin, Y is top to bottom

    // Blue bottom corner is the origin
    public static final Pose2d origin = new Pose2d();

    // Field
    public static final Distance fieldLength = Inches.of(690.875);
    public static final Distance fieldWidth = Inches.of(317);

    // Reef
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
    // 17 - 22, 6-11

    /**
     * Converts a tag ID with an offset into a reef position
     * @param tagId ID for the april tag
     * @param offset true for left offset, false for right offset
     * @return Pose2d of the determined reef position
     */

    public static final List<Pose2d> blueReefList = List.of(blueReefA, blueReefB, blueReefC, blueReefD, blueReefE, blueReefF, blueReefG, blueReefH, blueReefI, blueReefJ, blueReefK, blueReefL);
    public static final List<Pose2d> redReefList = List.of(redReefA, redReefB, redReefC, redReefD, redReefE, redReefF, redReefG, redReefH, redReefI, redReefJ, redReefK, redReefL);

    // Reef Coral
    public static final Distance levelOneHeight = Inches.of(18);
    public static final Distance levelTwoHeight = Inches.of(31.875);
    public static final Distance levelThreeHeight = Inches.of(47.625);
    public static final Distance levelFourHeight = Inches.of(72);

    // Reef Algae
    public static final Distance algaeLowHeight = levelTwoHeight.minus(Inches.of(6.25));
    public static final Distance algaeHighHeight = levelThreeHeight.minus(Inches.of(6.25));
    
    // Barge
    public static final Pose2d blueCageTop = new Pose2d(Inches.of(345.4375), Inches.of(285.875), Rotation2d.fromDegrees(180));
    public static final Pose2d blueCageMid = new Pose2d(Inches.of(345.4375), Inches.of(242.875), Rotation2d.fromDegrees(180));
    public static final Pose2d blueCageLow = new Pose2d(Inches.of(345.4375), Inches.of(200), Rotation2d.fromDegrees(180));

    public static final Pose2d redCageTop = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageTop.getMeasureY()), Rotation2d.fromDegrees(0));
    public static final Pose2d redCageMid = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageMid.getMeasureY()), Rotation2d.fromDegrees(0));
    public static final Pose2d redCageLow = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageLow.getMeasureY()), Rotation2d.fromDegrees(0));

    // Coral Station
    public static final Pose2d blueStationLow = new Pose2d(Inches.of(31.127), Inches.of(21.796), Rotation2d.fromDegrees(55));
    public static final Pose2d blueStationTop = new Pose2d(blueStationLow.getMeasureX(), fieldWidth.minus(blueStationLow.getMeasureY()), Rotation2d.fromDegrees(305));

    public static final Pose2d redStationLow = new Pose2d(fieldLength.minus(blueStationLow.getMeasureX()), blueStationLow.getMeasureY(), Rotation2d.fromDegrees(blueStationTop.getRotation().getDegrees() - 180));
    public static final Pose2d redStationTop = new Pose2d(fieldLength.minus(blueStationLow.getMeasureX()), fieldWidth.minus(blueStationLow.getMeasureY()), Rotation2d.fromDegrees(blueStationLow.getRotation().getDegrees() - 180));

    public static final List<Pose2d> blueCoralStationList = List.of(blueStationLow, blueStationTop);
    public static final List<Pose2d> redCoralStationList = List.of(redStationLow, redStationTop);

    // Offsets
    public static final Distance cageOffset = SwerveK.driveBaseLength.plus(SwerveK.driveBaseLength.div(2)); // Might need to tune this

    public static final Distance reefOffsetDistance = SwerveK.driveBaseLength.div(2);
    public static final Distance stationOffsetDistance = SwerveK.driveBaseLength.div(2);

    // Processor
    //^ processor location is on the edge of the arena carpet rather then the exact middle of the structure
    public static final Translation2d blueProcessor = new Translation2d(Inches.of(0), Inches.of(235.7255));
    public static final Translation2d redProcessor = new Translation2d(fieldWidth, fieldLength.minus(blueProcessor.getMeasureY()));
    
    /**
     * Creates a pose2d with 0 degrees from a given translation
     * @param translation Translation to get the x and y from
     * @return Pose with the translation's x and y, and 0 degrees for rotation
     */
    public static Pose2d getAsPose(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.fromDegrees(0));
    }
}
