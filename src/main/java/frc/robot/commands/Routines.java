package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Routines {

    // Prevent this class from being instantiated
    private Routines() {}

    public static Command stow(Elevator elevator, Intake intake, Algae algae) {
        return elevator.setPosition(Positions.STOW)
        .alongWith(
            intake.runOnce(intake::stop),
            algae.stow()
        )
        .withName("Stow Routine");
    }
    
    public static Command intakeCoral(Elevator elevator, Intake intake) {
        return Commands.either(
            intake.coralIntake(), 
            elevator.setPosition(Positions.INTAKE)
            .andThen(
                intake.coralIntake(),
                elevator.setPosition(Positions.STOW)
            ),
            intake::isSensorTripped
        ).withName("Intake Coral Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L1</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelOne(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L1)
        .andThen(intake
        .scoreLevelOne(),
            elevator.setPosition(Positions.STOW)
        ).withName("Score Coral L1 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L2</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelTwo(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L2)
        .andThen(
            intake.scoreLevelTwoThree(),
            elevator.setPosition(Positions.STOW)
        ).withName("Score Coral L2 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L3</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelThree(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L3)
        .andThen(
            intake.scoreLevelTwoThree(),
            elevator.setPosition(Positions.STOW)
        ).withName("Score Coral L3 Routine");
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an L4 branch.
     * @param elevator
     * @param intake
     */
    public static Command scoreCoralLevelFour(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L4)
        .andThen(
            intake.scoreLevelFour(),
            elevator.setPosition(Positions.STOW)
        ).withName("Score Coral L4 Routine");
    }
    
    /**
     * Command that sets the elevator and algae arm in position to be able to grab the <STRONG>low</STRONG> algae.
     * 
     * @return
     */
    public static Command algaeLowPosition(Elevator elevator, Algae algae) {
        return elevator.setPosition(Positions.ALGAE_LOW)
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
        return elevator.setPosition(Positions.ALGAE_HIGH)
        .alongWith(
            Commands.waitUntil(() -> elevator.getPosition().gte(ElevatorK.armThresholdHeight))
            .andThen(algae.algaePosition())
        ).withName("Algae High Routine");
    }

     /**
     * Creates a command to drive the robot to the nearest left-sdie reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public static Command alignToLeftReef(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListLeft : Field.blueReefListLeft);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return swerve.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.k180deg)
                ); 
            },
            Set.of(swerve)
        ).withName("Align Left Reef");
    }

    /**
     * Creates a command to drive the robot to the nearest right-side reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public static Command alignToRightReef(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListRight : Field.blueReefListRight);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return swerve.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.k180deg)
                ); 
            },
            Set.of(swerve)
        ).withName("Align Right Reef");
    }

    /**
     * Creates a command to drive the robot to the nearest coral station to it
     * @return driveToPoseCommand to drive to the nearest station on your side
     */
    public static Command alignToCoralStation(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d stationPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redCoralStationList : Field.blueCoralStationList);
                Translation2d stationOffset = new Translation2d(Field.stationOffsetDistance, Inches.of(0)).rotateBy(stationPose.getRotation());
                return swerve.driveToPoseCommand(            
                        stationPose.getMeasureX().plus(stationOffset.getMeasureX()),
                        stationPose.getMeasureY().plus(stationOffset.getMeasureY()),
                        stationPose.getRotation().plus(Rotation2d.k180deg)
                );
            }, 
            Set.of(swerve)
        ).withName("Align Coral Station");
    }

    /**
     * Creates a command to drive to the top cage of your side
     * @return driveToPoseCommand to drive to the top cage
     */

    public static Command alignToTopCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageTop = Robot.onRedAlliance() ? Field.redCageTop : Field.blueCageTop;
                return swerve.driveToPoseCommand(cageTop);
            },
            Set.of(swerve)
        ).withName("Align Top Cage");
    }

    /**
     * Creates a command to drive to the mid cage of your side
     * @return driveToPoseCommand to drive to the mid cage
     */
    public static Command alignToMidCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageMid = Robot.onRedAlliance() ? Field.redCageMid : Field.blueCageMid;
                return swerve.driveToPoseCommand(cageMid);
            },
            Set.of(swerve)
        ).withName("Align Mid Cage");
    }

    /**
     * Creates a command to drive to the low cage of your side
     * @return driveToPoseCommand to drive to the low cage
     */
    public static Command alignToLowCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageLow = Robot.onRedAlliance() ? Field.redCageLow : Field.blueCageLow;
                return swerve.driveToPoseCommand(cageLow);
            },
            Set.of(swerve)
        ).withName("Align Low Cage");
    }

    /**
     * Zeros all subsystems that need to be zeroed
     * @return
     */
    public static Command zeroAll(Elevator elevator, Algae algae, Climb climb) {
        return Commands.parallel(
            elevator.zero(),
            algae.zero(),
            climb.zero()
        ).withName("Zero Everything");
    }
}
