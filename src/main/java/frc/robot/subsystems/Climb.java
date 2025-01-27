package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

import frc.robot.lib.util.Util;
public class Climb extends SubsystemBase {
    
    TalonFX armMotor = new TalonFX(ClimbK.rMotorID);
    TalonFX armMotorFollow = new TalonFX(ClimbK.frMotorID);

    LinearVelocity linearVelocity = ClimbK.velocity;
    LinearAcceleration linearAcceleration = ClimbK.acceleration;
    //^Takes the number from Constant's ClimbK class and uses it as the motor ID^
    //Configure motors
    public Climb() {
        configTalons();
        configMotionMagic(linearVelocity,linearAcceleration);
    }
    
    
    
    private void configTalons() {
        
        
        
        Util.factoryReset(armMotor, armMotorFollow);
        Util.brakeMode(armMotor, armMotorFollow);
        
        armMotorFollow.setControl(new StrictFollower(ClimbK.rMotorID));
    }
    
    //MotionMagic
    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        //Set the Configs
        MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs();
        MotionMagicConfigs.MotionMagicCruiseVelocity = 0; //Find
        MotionMagicConfigs.MotionMagicAcceleration = 0; //Find
        armMotor.getConfigurator().apply(MotionMagicConfigs);

    }
    //Set Voltage
    // public void setVoltage(Voltage volts) {
    //     VoltageOut request = new VoltageOut(volts);
    //     armMotor.setControl(request);
    
    //Climb
    public Command deepclimb() {
        
        
        //Simply need to rotate arms until they need to stop.
        //Not sure if there will be a sensor that will be there or if it'll have to be a set degrees.
        
        // setVoltage(ClimbK.voltage);
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmDown);
        return runOnce(() -> armMotor.setControl(request));


    }
    //Release
    private void Release() {

    }
    //Stop
    


    
}
