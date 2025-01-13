// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    private static Field2d reefTest = new Field2d();
    
    public Robot() {
        SmartDashboard.putData("Arena: ", reefTest);

        //cycled clockwise A - K - I - G - E - C 
    //                     B - L - J - H - F - D 

        reefTest.getObject("Branch B").setPose(Field.redReefB);
        reefTest.getObject("Branch L").setPose(Field.redReefL);
        reefTest.getObject("Branch J").setPose(Field.redReefJ);
        // reefTest.getObject("Branch H").setPose(Field.blueReefH);
        // reefTest.getObject("Branch F").setPose(Field.blueReefF);
        // reefTest.getObject("Branch D").setPose(Field.blueReefD);

        reefTest.getObject("Branch A").setPose(Field.redReefA);
        reefTest.getObject("Branch K").setPose(Field.redReefK);
        reefTest.getObject("Branch I").setPose(Field.redReefI);
        // reefTest.getObject("Branch G").setPose(Field.blueReefE);
        // reefTest.getObject("Branch E").setPose(Field.blueReefG);
        // reefTest.getObject("Branch C").setPose(Field.blueReefC);
        
        reefTest.getObject("Origin").setPose(Field.origin);
        reefTest.getObject("Blue Center").setPose(Field.blueReefCenter.getX(), Field.blueReefCenter.getY(), new Rotation2d(0));
        
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
}
