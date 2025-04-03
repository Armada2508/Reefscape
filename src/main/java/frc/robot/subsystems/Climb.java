package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;
import frc.robot.lib.util.Util;

@Logged
public class Climb extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ClimbK.talonID);
    private final TalonFX talonFollow = new TalonFX(ClimbK.talonFollowID);
    private final Servo servoL = new Servo(ClimbK.servoLID);
    private final Servo servoR = new Servo(ClimbK.servoRID);

    public Climb() {
        configTalons();
    } 

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        talon.getConfigurator().apply(ClimbK.outputConfigs);
        talon.getConfigurator().apply(ClimbK.softLimitConfigs);
        talon.getConfigurator().apply(ClimbK.gearRatioConfig);
        talon.getConfigurator().apply(ClimbK.pidconfig);
        talon.getConfigurator().apply(ClimbK.currentConfigs);
        talon.getConfigurator().apply(new Slot1Configs().withKP(10));
        talonFollow.getConfigurator().apply(ClimbK.currentConfigs);
        talonFollow.setControl(new StrictFollower(ClimbK.talonID));
        Util.brakeMode(talon, talonFollow);
        talon.setPosition(ClimbK.stowAngle);
    }

    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond); 
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond); 
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    public Command servoCoast() {
        return runOnce(() -> {
            servoL.set(ClimbK.servoMax);
            servoR.set(ClimbK.servoMin);
        }).andThen(Commands.waitTime(ClimbK.servoAcutateTime)).withName("Servo Coast");
    }

    public Command servoRatchet() {
        return runOnce(() -> {
            servoL.set(ClimbK.servoMin);
            servoR.set(ClimbK.servoMax);
        }).andThen(Commands.waitTime(ClimbK.servoAcutateTime)).withName("Servo Ratchet");
    }

    /**
     * Sets the voltage of the motors.
     * @param volts voltage of the motor in volts.
     */
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> talon.setControl(request)).withName("Set Voltage");
    }

    /**
     * Preps the arm for climbing
     */
    public Command prep() {
        return servoCoast()
            .andThen(
                setVoltage(ClimbK.prepVoltage),
                Commands.waitUntil(() -> talon.getFault_ForwardSoftLimit().getValue())
            )
            .finallyDo(this::stop)
            .withName("Prep Climb");
    }

    public double getPositionDegrees() {
        return talon.getPosition().getValue().in(Degrees);
    }

    public double getVelocityDegrees() {
        return talon.getVelocity().getValue().in(DegreesPerSecond);
    }

    public Command stow() {
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.stowAngle);
        return servoCoast()
            .andThen(
                runOnce(() -> {
                    configMotionMagic(ClimbK.climbVelocity, ClimbK.climbAcceleration);
                    talon.setControl(request);
                }),
                Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.stowAngle, ClimbK.allowableError))
                .finallyDo(this::stop)
            ).withName("Climb Motion Magic");
    }

    /**
     * Climbs the deep cage using voltage
     */
    public Command climb() {
        return servoRatchet()
            .andThen(
                setVoltage(ClimbK.climbVoltage),
                Commands.waitUntil(() -> talon.getFault_ReverseSoftLimit().getValue())
            ).finallyDo(this::stop)
            .withName("Deep Climb");
    }
    
    /**
     * Climbs the deep cage using motion magic
     */
    public Command climbMotionMagic() {
        MotionMagicVelocityVoltage grip = new MotionMagicVelocityVoltage(ClimbK.gripVelocity).withAcceleration(ClimbK.gripAcceleration).withSlot(1);
        MotionMagicVoltage climb = new MotionMagicVoltage(ClimbK.minAngle);
        return servoRatchet()
            .andThen(
                runOnce(() -> {
                    talon.setControl(grip);
                }),
                Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.gripAngle, ClimbK.allowableError)),
                runOnce(() -> {
                    configMotionMagic(ClimbK.climbVelocity, ClimbK.climbAcceleration);
                    talon.setControl(climb);
                }),
                Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.minAngle, ClimbK.allowableError))
            ).finallyDo(this::stop).withName("Climb Motion Magic");
    }

    public Command moveFreely() {
        return servoCoast().andThen(() -> Util.coastMode(talon, talonFollow));
    }

    /**
     * Stops the climb motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public void zero() {
        talon.setPosition(ClimbK.stowAngle);
        Util.brakeMode(talon, talonFollow);
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

}
