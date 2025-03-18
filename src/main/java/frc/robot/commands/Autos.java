package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Constants.IntakeK;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Autos {
    // Prevent this class from being instantiated
    private Autos() {}

    public static SendableChooser<Command> initPathPlanner(Swerve swerve, Elevator elevator, Intake intake, Algae algae) {
        FollowPathCommand.warmupCommand().schedule();

        new EventTrigger("intake coral").onTrue(Routines.intakeCoral(elevator, intake).alongWith(Commands.print("INTAKING CORAL COMD")));
        NamedCommands.registerCommand("intake named", Routines.intakeCoral(elevator, intake));
        NamedCommands.registerCommand("score L1", Routines.scoreCoralLevelOne(elevator, intake));
        NamedCommands.registerCommand("score L2", Routines.scoreCoralLevelTwo(elevator, intake));
        NamedCommands.registerCommand("score L3", Routines.scoreCoralLevelThree(elevator, intake));
        NamedCommands.registerCommand("raise and score L4", elevator.setPosition(Positions.L4.close).andThen(Routines.scoreCoralLevelFour(elevator, intake)));
        NamedCommands.registerCommand("wait for score", Commands.waitUntil(() -> !intake.isSensorTripped()).andThen(Commands.waitSeconds(0.25)).withName("Wait for score"));
        NamedCommands.registerCommand("wait for intake", Commands.waitUntil(intake::isSensorTripped).andThen(Commands.waitTime(IntakeK.intakeAfterTrip)));
        NamedCommands.registerCommand("drive to coral", swerve.driveCommand(() -> 0.2, () -> 0, () -> 0, false, true).until(() -> elevator.getTimeOfFlightDistance() < 14.75).withTimeout(2).finallyDo(swerve::stop));

        new EventTrigger("score L4").onTrue(Commands.print("SCORING L4 REAL!!!!!!!!!!!!!!!!!!!!!").andThen(Commands.waitUntil(() -> elevator.nearHeight(Positions.L4.close))).andThen(Routines.scoreCoralLevelFour(elevator, intake)).withName("Auto score L4"));
        new EventTrigger("raise elevator to intake").onTrue(elevator.setPosition(Positions.INTAKE.close));
        new EventTrigger("raise elevator to L4").onTrue(elevator.setPosition(Positions.L4.close));
        new EventTrigger("stow elevator").onTrue(elevator.setPosition(Positions.STOW));

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Low Score 2 Coral Top");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
    
}
