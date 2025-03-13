package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Autos {
    // Prevent this class from being instantiated
    private Autos() {}

    public static SendableChooser<Command> initPathPlanner(Swerve swerve, Elevator elevator, Intake intake, Algae algae) {
        FollowPathCommand.warmupCommand().schedule();

        NamedCommands.registerCommand("intake coral", Routines.intakeCoral(elevator, intake));
        NamedCommands.registerCommand("score L1", Routines.scoreCoralLevelOne(elevator, intake));
        NamedCommands.registerCommand("score L2", Routines.scoreCoralLevelTwo(elevator, intake));
        NamedCommands.registerCommand("score L3", Routines.scoreCoralLevelThree(elevator, intake));
        NamedCommands.registerCommand("score L4", Routines.scoreCoralLevelFour(elevator, intake));
        
        new EventTrigger("raise elevator to intake").onTrue(elevator.setPosition(Positions.INTAKE));
        new EventTrigger("raise elevator to L4").onTrue(elevator.setPosition(Positions.L4));
        new EventTrigger("stow elevator").onTrue(elevator.setPosition(Positions.STOW));

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Low Score 4 Coral Top");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
    
}
