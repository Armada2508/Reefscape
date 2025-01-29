package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.lib.util.Util;
public class Climb extends SubsystemBase {
    
    private TalonFX armMotor = new TalonFX(ClimbK.armMotorID);
    private TalonFX armMotorFollow = new TalonFX(ClimbK.fArmMotorID);

   
    //^Takes the number from Constant's ClimbK class and uses it as the motor ID^
    //Configure motors
    public Climb() {
        configTalons();
        configMotionMagic(ClimbK.velocity, ClimbK.acceleration); 
    } 

    private void configTalons() {
        
        Util.factoryReset(armMotor, armMotorFollow);
        Util.brakeMode(armMotor, armMotorFollow);
        armMotor.getConfigurator().apply(ClimbK.softLimitConfigs);
        armMotorFollow.setControl(new StrictFollower(ClimbK.armMotorID));
    }
    
    //MotionMagic
    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        //Set the Configs
        MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs();
        MotionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond); 
        MotionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond); 
        armMotor.getConfigurator().apply(MotionMagicConfigs);

    }
    
    //Climb
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> armMotor.setControl(request));
    }

    public Command deepclimb() {
        
        return setVoltage(ClimbK.climbVoltage);
        
    }
    public Command deepClimbMM() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmDown);
        return runOnce(() -> armMotor.setControl(request));
    }
    //Release
    public Command Release() {
        return setVoltage(ClimbK.climbVoltage.unaryMinus());
    }
    //Stop
    public Command releaseMM() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmUp);
        return runOnce(() -> armMotor.setControl(request));
    }
}
