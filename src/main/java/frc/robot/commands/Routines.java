package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.subsystems.Algae;
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
        .andThen(algae.loweredPosition())
        .andThen(algae.algaePosition())
        .andThen(swerve.driveToPoseCommand(new Pose2d())) //!make it move backwards
        .andThen(algae.stow())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command removeAlgaeHigh(Elevator elevator, Algae algae, Swerve swerve) {
        return elevator.setPosition(Positions.ALGAE_HIGH)
        .andThen(algae.loweredPosition())
        .andThen(algae.algaePosition())
        .andThen(swerve.driveToPoseCommand(new Pose2d())) //!make it move backwards
        .andThen(algae.stow())
        .finallyDo(() -> elevator.setPosition(Positions.STOW));
    }

    public static Command alignCoralStation(Swerve swerve) {
        return swerve.alignToCoralStation();
    }
}
