package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

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
}
