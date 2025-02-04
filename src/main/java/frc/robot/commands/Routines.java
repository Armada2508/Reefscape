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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Routines {

    // Prevent this class from being instantiated
    private Routines() {}
    
    public static Command stowElevator(Elevator elevator) {
        return elevator.setPosition(Positions.STOW);
    }

    public static Command intakeCoral(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.INTAKE)
        .andThen(intake.coralIntake())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command scoreCoralLevelOne(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L1)
        .andThen(intake.scoreLevelOne())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command scoreCoralLevelTwo(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L2)
        .andThen(intake.scoreLevelTwoThree())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command scoreCoralLevelThree(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L3)
        .andThen(intake.scoreLevelTwoThree())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }
    
    public static Command scoreCoralLevelFour(Elevator elevator, Intake intake) {
        return elevator.setPosition(Positions.L4)
        .andThen(intake.scoreLevelFour())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command removeAlgaeLow(Elevator elevator, Algae algae, Swerve swerve) {
        return elevator.setPosition(Positions.ALGAE_LOW)
        .andThen(
            algae.loweredPosition(),
            swerve.driveCommand(() -> 0, () -> 1, () -> 0, false, false), //! Tune / Verify
            algae.algaePosition(),
            swerve.driveCommand(() -> 0, () -> 1, () -> 0, false, true), //! Tune / Verify
            algae.stow()
        )
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command removeAlgaeHigh(Elevator elevator, Algae algae, Swerve swerve) {
        return elevator.setPosition(Positions.ALGAE_HIGH)
        .andThen(
            algae.loweredPosition(),
            swerve.driveCommand(() -> 0, () -> 1, () -> 0, false, false), //! Tune / Verify
            algae.algaePosition(),
            swerve.driveCommand(() -> 0, () -> 1, () -> 0, false, true), //! Tune / Verify
            algae.stow()
        ) 
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
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
    public static Command alignToRightReef(Swerve swerve) {
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
    public static Command alignToCoralStation(Swerve swerve) {
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

    public static Command alignToTopCage(Swerve swerve) {
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
    public static Command alignToMidCage(Swerve swerve) {
        if (Robot.onRedAlliance()) { 
            return Commands.defer(
                () ->
                swerve.driveToPoseCommand(
                    Field.redCageMid.getMeasureX().plus(Field.cageOffset), 
                    Field.redCageMid.getMeasureY(), 
                    Field.redCageMid.getRotation()
                ), 
                Set.of(swerve)
            );  
        }
        return Commands.defer(
            () ->
            swerve.driveToPoseCommand(
                Field.blueCageMid.getMeasureX().minus(Field.cageOffset), 
                Field.blueCageMid.getMeasureY(), 
                Field.blueCageMid.getRotation()
            ), 
            Set.of(swerve)
        );
    }

    /**
     * Creates a command to drive to the low cage of your side
     * @return driveToPoseCommand to drive to the low cage
     */
    public static Command alignToLowCage(Swerve swerve) {
        if (Robot.onRedAlliance()) { 
            return Commands.defer(
                () ->
                swerve.driveToPoseCommand(
                    Field.redCageLow.getMeasureX().plus(Field.cageOffset), 
                    Field.redCageLow.getMeasureY(), 
                    Field.redCageLow.getRotation()
                ), 
                Set.of(swerve)
            );  
        }
        return Commands.defer(
            () ->
            swerve.driveToPoseCommand(
                Field.blueCageLow.getMeasureX().minus(Field.cageOffset), 
                Field.blueCageLow.getMeasureY(), 
                Field.blueCageLow.getRotation()
            ), 
            Set.of(swerve)
        );
    }
    public static Command climb(Climb climb) {
        return climb.deepclimb();
    }
    public static Command climbMotionMagic(Climb climb) {
        return climb.deepClimbMotionMagic();
    }
    public static Command zero(Climb climb) {
        return climb.zero();
    }
}
