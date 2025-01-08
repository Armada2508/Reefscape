package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Field {
    //TODO double check all values in at some point, good chance something could be wrong here, rookie task?
    // X is left and right from origin, Y is top to bottom
    // Blue is origin
    public static final Pose2d origin = new Pose2d();

    // Field
    public static final Distance fieldLength = Inches.of(690.875);
    public static final Distance fieldWidth = Inches.of(317);

    // Reef
    public static final Translation2d blueReefA = new Translation2d();
    public static final Translation2d blueReefB = new Translation2d();
    public static final Translation2d blueReefC = new Translation2d();
    public static final Translation2d blueReefD = new Translation2d();
    public static final Translation2d blueReefE = new Translation2d();
    public static final Translation2d blueReefF = new Translation2d();
    public static final Translation2d blueReefG = new Translation2d();
    public static final Translation2d blueReefH = new Translation2d();
    public static final Translation2d blueReefI = new Translation2d();
    public static final Translation2d blueReefJ = new Translation2d();
    public static final Translation2d blueReefK = new Translation2d();
    public static final Translation2d blueReefL = new Translation2d();

    public static final Translation2d redReefA = new Translation2d();
    public static final Translation2d redReefB = new Translation2d();
    public static final Translation2d redReefC = new Translation2d();
    public static final Translation2d redReefD = new Translation2d();
    public static final Translation2d redReefE = new Translation2d();
    public static final Translation2d redReefF = new Translation2d();
    public static final Translation2d redReefG = new Translation2d();
    public static final Translation2d redReefH = new Translation2d();
    public static final Translation2d redReefI = new Translation2d();
    public static final Translation2d redReefJ = new Translation2d();
    public static final Translation2d redReefK = new Translation2d();
    public static final Translation2d redReefL = new Translation2d();

    // Barge
    public static final Translation2d blueCageTop = new Translation2d(Inches.of(345.4375), Inches.of(285.875));
    public static final Translation2d blueCageMid = new Translation2d(Inches.of(345.4375), Inches.of(242.875));
    public static final Translation2d blueCageLow = new Translation2d(Inches.of(345.4375), Inches.of(200));

    public static final Translation2d redCageTop = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageTop.getMeasureY()));
    public static final Translation2d redCageMid = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageMid.getMeasureY()));
    public static final Translation2d redCageLow = new Translation2d(Inches.of(345.4375), fieldWidth.minus(blueCageLow.getMeasureY()));

    // Coral Station
    public static final Translation2d blueStationTop = new Translation2d();
    public static final Translation2d blueStationLow = new Translation2d(Inches.of(130.71), Inches.of(81.69));

    public static final Translation2d redStationTop = new Translation2d();
    public static final Translation2d redStationLow = new Translation2d();

    // Processor

    
}
