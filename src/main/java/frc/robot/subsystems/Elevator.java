package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

public class Elevator extends SubsystemBase {
    private final TalonFX talon = new TalonFX(ElevatorK.elevatorID);
    private final TalonFX talonFollow = new TalonFX(ElevatorK.followID);

    public Elevator() {
        configMotionMagic(ElevatorK.velocity, ElevatorK.acceleration); //! Find?
        configTalon();
    }

    private void configTalon() {
        Util.factoryReset(talon);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(ElevatorK.elevatorConfig); //applies gear ratio, need to apply to follower?
        talon.getConfigurator().apply(ElevatorK.pidConfig); // applies PID constants, still need to tunee
    }

    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(MetersPerSecond); //! Find
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(MetersPerSecondPerSecond); //! Find
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this classes constants file.
     * @param position position of the elavator to move to
     */
    public Command setPosition(ElevatorK.Positions position) {
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.toRotations(position.level, ElevatorK.gearRatio, Inches.of(0))); //what wheel?
        return runOnce(() -> talon.setControl(request))
        .andThen(Commands.waitUntil(() -> getPosition().isNear(position.level, ElevatorK.allowableError))); // stop when we reach set position
    }

    public Distance getPosition() {
        return Encoder.toDistance(talon.getPosition().getValue(), ElevatorK.gearRatio, ElevatorK.wheelDiameter);
    }

    /**
     * 
     * @param speed speed of the motor in volts
     */
    public void setVoltage(Voltage speed) {
        VoltageOut request = new VoltageOut(speed);
        talon.setControl(request);
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }
}
