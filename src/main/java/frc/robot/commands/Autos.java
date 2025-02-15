package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorK;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Autos {
    // Prevent this class from being instantiated
    private Autos() {}

    public static SendableChooser<Command> initPathPlanner(Swerve swerve, Elevator elevator, Intake intake) {
        FollowPathCommand.warmupCommand().schedule();
        // These are left here for reference for when we have our subsystems to coordinate in Auto

        NamedCommands.registerCommand("intake", Routines.intakeCoral(elevator, intake));
        NamedCommands.registerCommand("score L1", Routines.scoreCoralLevelOne(elevator, intake));
        NamedCommands.registerCommand("score L2", Routines.scoreCoralLevelTwo(elevator, intake));
        NamedCommands.registerCommand("score L3", Routines.scoreCoralLevelThree(elevator, intake));
        NamedCommands.registerCommand("score L4", Routines.scoreCoralLevelFour(elevator, intake));

        
        new EventTrigger("raise elevator to intake").onTrue(elevator.setPosition(ElevatorK.Positions.INTAKE));


        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Score 2 CORAL");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
    
}
