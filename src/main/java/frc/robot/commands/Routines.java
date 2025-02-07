package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Routines {

    // Prevent this class from being instantiated
    private Routines() {}
    
    public Command intakeCoral(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.INTAKE)
        .andThen(intake.coralIntake())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L1</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public Command scoreCoralLevelOne(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L1)
        .andThen(intake.scoreLevelOne())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L2</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public Command scoreCoralLevelTwo(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L2)
        .andThen(intake.scoreLevelTwoThree())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an <STRONG>L3</STRONG> branch.
     * @param elevator
     * @param intake
     */
    public Command scoreCoralLevelThree(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L3)
        .andThen(intake.scoreLevelTwoThree())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    /**
     * Command that raises the elevator for the intake to score the coral on an L4 branch.
     * @param elevator
     * @param intake
     */
    public Command scoreCoralLevelFour(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L4)
        .andThen(intake.scoreLevelFour())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    /**
     * Command that sets the elevator and algae arm in position to be able to grab the <STRONG>low</STRONG> algae.
     * 
     * @return
     */
    public Command algaeLowPosition(Elevator elevator, Algae algae) {
        return elevator.setPosition(Positions.ALGAE_LOW)
        .alongWith(
            algae.loweredPosition()
        );
    }
    /**
     * Command that sets the elevator and algae arm in position to be able to grab the <STRONG>high</STRONG> algae.
     * 
     * @return
     */
    public Command algaeHighPosition(Elevator elevator, Algae algae) {
        return elevator.setPosition(Positions.ALGAE_HIGH)
        .alongWith(
            algae.loweredPosition()
        );
    }

     /**
     * Creates a command to drive the robot to the nearest left-sdie reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public Command alignToLeftReef(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListLeft : Field.blueReefListLeft);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return swerve.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ); 
            },
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive the robot to the nearest right-side reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public Command alignToRightReef(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListRight : Field.blueReefListRight);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return swerve.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ); 
            },
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive the robot to the nearest coral station to it
     * @return driveToPoseCommand to drive to the nearest station on your side
     */
    public Command alignToCoralStation(Swerve swerve) {
        return Commands.defer(
            () -> {
                System.out.println("coral station alignment called");
                Pose2d stationPose = swerve.getPose().nearest(Robot.onRedAlliance() ? Field.redCoralStationList : Field.blueCoralStationList);
                Translation2d stationOffset = new Translation2d(Field.stationOffsetDistance, Inches.of(0)).rotateBy(stationPose.getRotation());
                return swerve.driveToPoseCommand(            
                        stationPose.getMeasureX().plus(stationOffset.getMeasureX()),
                        stationPose.getMeasureY().plus(stationOffset.getMeasureY()),
                        stationPose.getRotation().plus(Rotation2d.fromDegrees(180))
                );
            }, 
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive to the top cage of your side
     * @return driveToPoseCommand to drive to the top cage
     */

    public Command alignToTopCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageTop = Robot.onRedAlliance() ? Field.redCageTop : Field.blueCageTop;
                return swerve.driveToPoseCommand(
                    cageTop.getMeasureX().minus(Field.cageOffset), 
                    cageTop.getMeasureY(), 
                    cageTop.getRotation()
                );
            },
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive to the mid cage of your side
     * @return driveToPoseCommand to drive to the mid cage
     */
    public Command alignToMidCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageMid = Robot.onRedAlliance() ? Field.redCageTop : Field.blueCageTop;
                return swerve.driveToPoseCommand(
                    cageMid.getMeasureX().minus(Field.cageOffset), 
                    cageMid.getMeasureY(), 
                    cageMid.getRotation()
                );
            },
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive to the low cage of your side
     * @return driveToPoseCommand to drive to the low cage
     */
    public Command alignToLowCage(Swerve swerve) {
        return Commands.defer(
            () -> {
                Pose2d cageLow = Robot.onRedAlliance() ? Field.redCageTop : Field.blueCageTop;
                return swerve.driveToPoseCommand(
                    cageLow.getMeasureX().minus(Field.cageOffset), 
                    cageLow.getMeasureY(), 
                    cageLow.getRotation()
                );
            },
            Set.of(swerve)
        );
    }
}
