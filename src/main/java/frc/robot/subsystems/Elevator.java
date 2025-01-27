package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
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

@Logged
public class Elevator extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ElevatorK.elevatorID);
    private final TalonFX talonFollow = new TalonFX(ElevatorK.followID);

    public Elevator() {
        configTalons();
        configMotionMagic(ElevatorK.velocity, ElevatorK.acceleration);
    }

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(ElevatorK.softwareLimitConfig);
        talon.getConfigurator().apply(ElevatorK.gearRatioConfig);
        talon.getConfigurator().apply(ElevatorK.pidConfig);
    }

    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = (velocity.in(MetersPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters));
        motionMagicConfigs.MotionMagicAcceleration = (acceleration.in(MetersPerSecondPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters));
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this classes constants file.
     * @param position position of the elavator to move to
     */
    public Command setPosition(ElevatorK.Positions position) {
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.linearToAngular(position.level.div(ElevatorK.stageCount), ElevatorK.sprocketDiameter));
        return runOnce(() -> talon.setControl(request))
        .andThen(Commands.waitUntil(() -> getPosition().isNear(position.level, ElevatorK.allowableError))) // end command when we reach set position
        .withName("Set Position"); 
    }

    /**
     * Gets the current height of the elevator
     * @return Height of the elevator as a Distance
     */
    public Distance getPosition() {
        return Encoder.angularToLinear(talon.getPosition().getValue().times(ElevatorK.stageCount), ElevatorK.sprocketDiameter);
    }

    /**
     * Sets the volts of the motors
     * @param speed speed of the motor in volts
     */
    public Command setVoltage(Voltage volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts))).withName("Set Voltage");
    }

    /**
     * Stops both motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }
    
}
