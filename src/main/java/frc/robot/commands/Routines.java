package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Constants.SwerveK;
import frc.robot.Field;
import frc.robot.Field.ReefSide;
import frc.robot.Robot;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Routines {

    // Prevent this class from being instantiated
    private Routines() {}

    public static Command stow(Elevator elevator, Intake intake, Climb climb) {
        return elevator.setPositionCommand(Positions.STOW)
            .alongWith(
            intake.runOnce(intake::stop),
            climb.stow()
        )
        .withName("Stow Routine");
    }
    
    public static Command intakeCoral(Elevator elevator, Intake intake) {
        return elevator.setPositionCommand(Positions.INTAKE).withDeadline(intake.intakeCoral())
        .andThen(
            Commands.defer(() -> elevator.setPositionCommand(elevator.getPosition().plus(ElevatorK.intakeBumpHeight)), Set.of(elevator)),
            intake.runOnce(() -> intake.stop()),
            elevator.setPositionCommand(Positions.STOW),
            intake.secureCoral()
        ).withName("Intake Coral Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L1</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelOne(Elevator elevator, Intake intake) {
        return intake.scoreLevelOne().andThen(elevator.setPositionCommand(Positions.STOW))
        .withName("Score Coral L1 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L2</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelTwo(Elevator elevator, Intake intake) {
        return intake.scoreLevelTwoThree().andThen(elevator.setPositionCommand(Positions.STOW))
        .withName("Score Coral L2 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L3</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelThree(Elevator elevator, Intake intake) {
        return intake.scoreLevelTwoThree().andThen(elevator.setPositionCommand(Positions.STOW))
        .withName("Score Coral L3 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an L4 branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelFour(Elevator elevator, Intake intake) {
        return intake.scoreLevelFour().andThen(elevator.setPositionCommand(Positions.STOW))
        .withName("Score Coral L4 Routine");
    }
    
    /**
     * Command that sets the elevator and algae arm in position to be able to grab the <STRONG>low</STRONG> algae.
     * 
     * @return
     */
    public static Command algaeLowPosition(Elevator elevator, Algae algae) {
        return elevator.setPositionCommand(Positions.ALGAE_LOW)
        .alongWith(
            Commands.waitUntil(() -> elevator.getPosition().gte(ElevatorK.armThresholdHeight))
            .andThen(algae.loweredPosition())
        ).withName("Algae Low Routine");
    }
    /**
     * Command that sets the elevator and algae arm in position to be able to grab the <STRONG>high</STRONG> algae.
     * 
     * @return
     */
    public static Command algaeHighPosition(Elevator elevator, Algae algae) {
        return elevator.setPositionCommand(Positions.ALGAE_HIGH)
        .alongWith(
            Commands.waitUntil(() -> elevator.getPosition().gte(ElevatorK.armThresholdHeight))
            .andThen(algae.algaePosition())
        ).withName("Algae High Routine");
    }

     /**
     * Creates a command to drive the robot to the nearest reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public static Command alignToReef(ReefSide side, Swerve swerve) {
        return Commands.either(swerve.alignToPosePID(() -> {
            Pose2d reefPose = swerve.getPose().nearest(Robot.onRedAlliance() ? side.redReef : side.blueReef);
            Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.zero()).rotateBy(reefPose.getRotation());
            return new Pose2d(reefPose.getTranslation().plus(reefOffset), reefPose.getRotation().plus(Rotation2d.k180deg));
        }), Commands.print("Haven't initalized odometry yet!"), swerve::initializedOdometryFromVision).withName("Align " + side + " Reef");
    }

    /**
     * Creates a command to drive the robot to the nearest coral station to it
     * @return driveToPoseCommand to drive to the nearest station on your side
     */
    public static Command alignToCoralStation(Swerve swerve) {
        return swerve.alignToPosePID(() -> {
            Pose2d stationPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redCoralStationList : Field.blueCoralStationList);
            Translation2d stationOffset = new Translation2d(Field.stationOffsetDistance, Inches.zero()).rotateBy(stationPose.getRotation());
            return new Pose2d(stationPose.getTranslation().plus(stationOffset), stationPose.getRotation().plus(Rotation2d.k180deg));
        }).withName("Align Coral Station");
    }

    /**
     * Creates a command to drive to the cage of your side
     * @return driveToPoseCommand to drive to the mid cage
     */
    public static Command alignToCage(Swerve swerve) {
        return Commands.either(
            swerve.alignToPosePID(() -> {
                var cage = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redCages : Field.blueCages);
                Distance x = Robot.onRedAlliance() ? Inches.of(8.75) : Inches.of(-8.75);
                Distance y = Robot.onRedAlliance() ? Inches.of(-6.375) : Inches.of(6.375);
                return new Pose2d(
                    new Translation2d(cage.getMeasureX().plus(x), cage.getMeasureY().plus(y)), 
                    cage.getRotation().plus(Rotation2d.fromDegrees(-45)));
            }, SwerveK.climbTranslationConstraints),
            Commands.print("Haven't initalized odometry yet!"),
            swerve::initializedOdometryFromVision
        ).withName("Align Cage");
    }

}
