package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.lib.util.Util;
public class Climb extends SubsystemBase {
    
    TalonFX armMotor = new TalonFX(ClimbK.armMotorID);
    TalonFX armMotorFollow = new TalonFX(ClimbK.fArmMotorID);

   
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
        MotionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(DegreesPerSecond); 
        MotionMagicConfigs.MotionMagicAcceleration = acceleration.in(DegreesPerSecondPerSecond); 
        armMotor.getConfigurator().apply(MotionMagicConfigs);

    }
    
    //Climb
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> armMotor.setControl(request));
    }

    public Command deepclimb() {
        // MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmDown);
        return setVoltage(ClimbK.voltagePos);
        //Simply sets the voltage to positive to go forward
    }
    //Release
    public Command Release() {
        return setVoltage(ClimbK.voltagePos.unaryMinus());
    }
    //Stop
    
}
