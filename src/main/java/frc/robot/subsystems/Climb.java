package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

import frc.robot.lib.util.Util;
public class Climb extends SubsystemBase {
    
    TalonFX armMotor = new TalonFX(ClimbK.rMotorID);
    TalonFX armMotorFollow = new TalonFX(ClimbK.frMotorID);

   
    //^Takes the number from Constant's ClimbK class and uses it as the motor ID^
    //Configure motors
    public Climb() {
        configTalons();
        configMotionMagic(ClimbK.velocity, ClimbK.acceleration);
    }
    
    
    
    private void configTalons() {
        
        Util.factoryReset(armMotor, armMotorFollow);
        Util.brakeMode(armMotor, armMotorFollow);
        
        armMotorFollow.setControl(new StrictFollower(ClimbK.rMotorID));
    }
    
    //MotionMagic
    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        //Set the Configs
        MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs();
        MotionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(DegreesPerSecond); 
        MotionMagicConfigs.MotionMagicAcceleration = acceleration.in(DegreesPerSecondPerSecond); 
        armMotor.getConfigurator().apply(MotionMagicConfigs);

    }
    //Set Voltage
    // public void setVoltage(Voltage volts) {
    //     VoltageOut request = new VoltageOut(volts);
    //     armMotor.setControl(request);
    
    //Climb
    public Command deepclimb() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmDown);
        return runOnce(() -> armMotor.setControl(request));
        
    }
    //Release
    public Command Release() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmUp);
        return runOnce(() -> armMotor.setControl(request));
    }
    //Stop
    
}
