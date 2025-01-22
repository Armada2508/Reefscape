package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Autos {

    // Prevent this class from being instantiated
    private Autos() {}

    public static SendableChooser<Command> initPathPlanner(Swerve swerve) {
        FollowPathCommand.warmupCommand().schedule();
        // These are left here for reference for when we have our subsystems to coordinate in Auto
        NamedCommands.registerCommand("score coral", Commands.print("Scoring CORAL!").andThen(Commands.waitSeconds(2)));
        NamedCommands.registerCommand("intake coral", Commands.print("Intaking CORAL!").andThen(Commands.waitSeconds(2)));
        new EventTrigger("raise elevator").onTrue(Commands.print("Raising Elevator!"));
        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Score 2 CORAL");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
    
}
