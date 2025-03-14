package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.SwerveK;

public class Field {
    // TODO: double check all values in at some point, good chance something could be wrong here, rookie task?
    //^  X is left and right from origin, Y is top to bottom

    // Blue bottom corner is the origin
    public static final Pose2d origin = Pose2d.kZero;

    // Field
    public static final Distance fieldLength = Inches.of(690.875);
    public static final Distance fieldWidth = Inches.of(317);

    // Reef
    public static final Translation2d blueReefCenter = new Translation2d(Inches.of(176.2768), Inches.of(158.4991));

    public static final Pose2d blueReefB = new Pose2d(new Translation2d(Inches.of(143.531), Inches.of(152.03)), Rotation2d.k180deg);
    public static final Pose2d blueReefL = new Pose2d(blueReefB.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));
    public static final Pose2d blueReefJ = new Pose2d(blueReefL.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(60));
    public static final Pose2d blueReefH = new Pose2d(blueReefJ.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.kZero);
    public static final Pose2d blueReefF = new Pose2d(blueReefH.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60));
    public static final Pose2d blueReefD = new Pose2d(blueReefF.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-120));

    public static final Pose2d blueReefA = new Pose2d(Inches.of(143.531), Inches.of(164.95), Rotation2d.k180deg);
    public static final Pose2d blueReefK = new Pose2d(blueReefA.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120));
    public static final Pose2d blueReefI = new Pose2d(blueReefK.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(60));
    public static final Pose2d blueReefG = new Pose2d(blueReefI.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.kZero);
    public static final Pose2d blueReefE = new Pose2d(blueReefG.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60));
    public static final Pose2d blueReefC = new Pose2d(blueReefE.getTranslation().rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-120));

    public static final Pose2d redReefA = new Pose2d(fieldLength.minus(blueReefA.getMeasureX()), blueReefA.getMeasureY(), Rotation2d.kZero);
    public static final Pose2d redReefK = new Pose2d(fieldLength.minus(blueReefK.getMeasureX()), blueReefK.getMeasureY(), Rotation2d.fromDegrees(60));
    public static final Pose2d redReefI = new Pose2d(fieldLength.minus(blueReefI.getMeasureX()), blueReefI.getMeasureY(), Rotation2d.fromDegrees(120));
    public static final Pose2d redReefG = new Pose2d(fieldLength.minus(blueReefG.getMeasureX()), blueReefG.getMeasureY(), Rotation2d.k180deg);
    public static final Pose2d redReefE = new Pose2d(fieldLength.minus(blueReefE.getMeasureX()), blueReefE.getMeasureY(), Rotation2d.fromDegrees(-120));
    public static final Pose2d redReefC = new Pose2d(fieldLength.minus(blueReefC.getMeasureX()), blueReefC.getMeasureY(), Rotation2d.fromDegrees(-60));

    public static final Pose2d redReefB = new Pose2d(fieldLength.minus(blueReefB.getMeasureX()), blueReefB.getMeasureY(), Rotation2d.kZero);
    public static final Pose2d redReefL = new Pose2d(fieldLength.minus(blueReefL.getMeasureX()), blueReefL.getMeasureY(), Rotation2d.fromDegrees(60));
    public static final Pose2d redReefJ = new Pose2d(fieldLength.minus(blueReefJ.getMeasureX()), blueReefJ.getMeasureY(), Rotation2d.fromDegrees(120));
    public static final Pose2d redReefH = new Pose2d(fieldLength.minus(blueReefH.getMeasureX()), blueReefH.getMeasureY(), Rotation2d.k180deg);
    public static final Pose2d redReefF = new Pose2d(fieldLength.minus(blueReefF.getMeasureX()), blueReefF.getMeasureY(), Rotation2d.fromDegrees(-120));
    public static final Pose2d redReefD = new Pose2d(fieldLength.minus(blueReefD.getMeasureX()), blueReefD.getMeasureY(), Rotation2d.fromDegrees(-60));

    public static final List<Pose2d> blueReefListLeft = List.of(blueReefA, blueReefC, blueReefE, blueReefG, blueReefI, blueReefK);
    public static final List<Pose2d> blueReefListRight = List.of(blueReefB, blueReefD, blueReefF, blueReefH, blueReefJ, blueReefL);

    public static final List<Pose2d> redReefListRight = List.of(redReefA, redReefC, redReefE, redReefG, redReefI, redReefK);
    public static final List<Pose2d> redReefListLeft = List.of(redReefB, redReefD, redReefF, redReefH, redReefJ, redReefL);

    // Reef Coral
    public static final Distance levelOneHeight = Inches.of(18);
    public static final Distance levelTwoHeight = Inches.of(31.875);
    public static final Distance levelThreeHeight = Inches.of(47.625);
    public static final Distance levelFourHeight = Inches.of(72);

    // Reef Algae
    public static final Distance algaeLowHeight = levelTwoHeight.minus(Inches.of(6.25));
    public static final Distance algaeHighHeight = levelThreeHeight.minus(Inches.of(6.25));
    
    // Barge
    public static final Pose2d blueCageTop = new Pose2d(Inches.of(345.4375), Inches.of(285.875), Rotation2d.k180deg);
    public static final Pose2d blueCageMid = new Pose2d(Inches.of(345.4375), Inches.of(242.875), Rotation2d.k180deg);
    public static final Pose2d blueCageLow = new Pose2d(Inches.of(345.4375), Inches.of(200), Rotation2d.k180deg);

    public static final Pose2d redCageTop = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageTop.getMeasureY()), Rotation2d.kZero);
    public static final Pose2d redCageMid = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageMid.getMeasureY()), Rotation2d.kZero);
    public static final Pose2d redCageLow = new Pose2d(Inches.of(345.4375), fieldWidth.minus(blueCageLow.getMeasureY()), Rotation2d.kZero);

    // Coral Station
    public static final Pose2d blueStationLow = new Pose2d(Inches.of(32.875), Inches.of(25.573), Rotation2d.fromDegrees(55));
    public static final Pose2d blueStationTop = new Pose2d(blueStationLow.getMeasureX(), fieldWidth.minus(blueStationLow.getMeasureY()), Rotation2d.fromDegrees(305));

    public static final Pose2d redStationLow = new Pose2d(fieldLength.minus(blueStationLow.getMeasureX()), blueStationLow.getMeasureY(), Rotation2d.fromDegrees(blueStationTop.getRotation().getDegrees() - 180));
    public static final Pose2d redStationTop = new Pose2d(fieldLength.minus(blueStationLow.getMeasureX()), fieldWidth.minus(blueStationLow.getMeasureY()), Rotation2d.fromDegrees(blueStationLow.getRotation().getDegrees() - 180));

    public static final List<Pose2d> blueCoralStationList = List.of(blueStationLow, blueStationTop);
    public static final List<Pose2d> redCoralStationList = List.of(redStationLow, redStationTop);

    // Offsets
    public static final Distance reefOffsetDistance = SwerveK.driveBaseLength.div(2).plus(Inches.of(0));
    public static final Distance stationOffsetDistance = SwerveK.driveBaseLength.div(2).plus(Inches.of(0));

    // Processor
    //^ processor location is on the edge of the arena carpet rather then the exact middle of the structure
    public static final Translation2d blueProcessor = new Translation2d(Inches.zero(), Inches.of(235.7255));
    public static final Translation2d redProcessor = new Translation2d(fieldWidth, fieldLength.minus(blueProcessor.getMeasureY()));

    public enum ReefSide {
        RIGHT(Field.blueReefListRight, Field.redReefListRight),
        LEFT(Field.blueReefListLeft, Field.redReefListLeft);

        public final List<Pose2d> blueReef;
        public final List<Pose2d> redReef;
        
        private ReefSide(List<Pose2d> blueReef, List<Pose2d> redReef) {
            // The passed in lists should already be immutable
            this.blueReef = blueReef;
            this.redReef = redReef;
        }
    }

    public enum Cage {
        TOP(Field.blueCageTop, Field.redCageTop),
        MIDDLE(Field.blueCageMid, Field.redCageMid),
        BOTTOM(Field.blueCageLow, Field.redCageLow);

        public final Pose2d bluePose;
        public final Pose2d redPose;

        private Cage(Pose2d bluePose, Pose2d redPose) {
            this.bluePose = bluePose;
            this.redPose = redPose;
        }
    }

    // Hi! Welcome to introspection :) https://www.oracle.com/technical-resources/articles/java/javareflection.html
    /**
     * For testing purposes only, call me to dump all Pose2d field constants onto network tables under /Field
     */
    public static void dumpToNT() {
        var table = NetworkTableInstance.getDefault().getTable("Field");
        var fields = Field.class.getDeclaredFields();
        for (var field : fields) {
            if (field.getType().equals(Pose2d.class)) {
                try {
                    table.getStructTopic(field.getName(), Pose2d.struct).publish().accept((Pose2d) field.get(null));
                    if (field.getName().contains("blueReef")) {
                        table.getStructTopic(field.getName() + " Offset", Pose2d.struct)
                            .publish()
                            .accept(getPoseWithNormalOffset((Pose2d) field.get(null), reefOffsetDistance));
                    }
                    if (field.getName().contains("Station")) {
                        table.getStructTopic(field.getName() + " Offset", Pose2d.struct)
                            .publish()
                            .accept(getPoseWithNormalOffset((Pose2d) field.get(null), stationOffsetDistance));
                    }
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    e.printStackTrace();
                };
            }
        }
    }

    public static Pose2d getPoseWithNormalOffset(Pose2d pose, Distance offset) {
        Translation2d normalOffset = new Translation2d(offset, Inches.zero()).rotateBy(pose.getRotation());
        return new Pose2d(pose.getTranslation().plus(normalOffset), pose.getRotation());
    }

}
