package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.lib.util.Util;
public class Climb extends SubsystemBase {
    
    private final TalonFX armMotor = new TalonFX(ClimbK.armMotorID);
    private final TalonFX armMotorFollow = new TalonFX(ClimbK.fArmMotorID);
    //^Takes the number from Constant's ClimbK class and uses it as the motor ID^
    //Configure motors
    public Climb() {
        configTalons();
        configMotionMagic(ClimbK.velocity, ClimbK.acceleration); 
    } 
    private void configTalons() {
        
        Util.factoryReset(armMotor, armMotorFollow);
        Util.brakeMode(armMotor, armMotorFollow);
        armMotor.getConfigurator().apply(ClimbK.hardLimitSwitchConfigs);
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
    /**
     * Sets the volts of the motors
     * @param volts speed of the motor in volts
     */
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> armMotor.setControl(request))
        .withName("Set Voltage");
    }
    /**
     * Command for activating the climbing arms, suspending the robot in midair via use of a cage.
     * 
     */
    public Command deepclimb() {
        
        return setVoltage(ClimbK.climbVoltage)
        .withName("Climbed");
        
    }
    /**
     * Motion magic version of the deepClimb command.
     * 
     */
    public Command deepClimbMotionMagic() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmDown);
        return runOnce(() -> armMotor.setControl(request))
        .andThen(Commands.waitUntil(() -> armMotor.getPosition().getValue().isNear(ClimbK.climbArmDown, ClimbK.allowableError)))
        .withName("Climbed with Motion Magic");
    }
    /** 
     * Command for bringing the arms back up, setting the robot (in a climbing state) back on the ground
     * 
     */
    public Command release() {
        return setVoltage(ClimbK.climbVoltage.unaryMinus())
        .withName("Released");
    }
    /** Motion magic version of the Release command.
     */ 
    public Command releaseMotionMagic() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.climbArmUp);
        return runOnce(() -> armMotor.setControl(request))
        .andThen(Commands.waitUntil(() -> armMotor.getPosition().getValue().isNear(ClimbK.climbArmUp, ClimbK.allowableError)))
        .withName("Release Motion Magic");
    }
    /**
     * 
     * Stops the arm motors
     */
    public void stop() {
        armMotor.setControl(new NeutralOut());
    }
}
