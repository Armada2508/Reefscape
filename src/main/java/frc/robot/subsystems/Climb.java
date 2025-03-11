package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;
import frc.robot.lib.util.Util;

@Logged
public class Climb extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ClimbK.talonID);
    private final TalonFX talonFollow = new TalonFX(ClimbK.talonFollowID);
    private final Debouncer homingDebouncer = new Debouncer(ClimbK.homingTime.in(Seconds));
    private final Servo servo = new Servo(ClimbK.servoID);
    private boolean isZeroed = false;

    public Climb() {
        configMotionMagic(ClimbK.maxVelocity, ClimbK.maxAcceleration); 
        configTalons();
    } 

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talon.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        talon.getConfigurator().apply(ClimbK.softLimitConfigs);
        talon.getConfigurator().apply(ClimbK.gearRatioConfig);
        talon.getConfigurator().apply(ClimbK.pidconfig);
        talonFollow.setControl(new StrictFollower(ClimbK.talonID));
    }

    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond); 
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond); 
        talon.getConfigurator().apply(motionMagicConfigs);
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
     * Climbs the deep cage using voltage
     */
     public Command deepclimb() {
        servo.set(1);
        servo.setAngle(ClimbK.servoActiveAngle.in(Degree));
        SmartDashboard.putNumber("Servo Angle", servo.get());
        return Commands.either(
            setVoltage(ClimbK.climbVoltage)
            .andThen(Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.maxAngle, ClimbK.allowableError))),
            Commands.print("Climb not zeroed!"),
            () -> isZeroed
        ).withName("Deep Climb");
    }
    
    /**
     * Climbs the deep cage using motion magic
     */
    public Command deepClimbMotionMagic() {
        servo.set(1);
        servo.setAngle(ClimbK.servoActiveAngle.in(Degree));
        MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.maxAngle);
        return Commands.either(
            Commands.runOnce(() -> talon.setControl(request))
            .andThen(Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.maxAngle, ClimbK.allowableError))),
            Commands.print("Climb not zeroed!"),
            () -> isZeroed
        )
        .withName("Deep Climb Motion Magic");
        
    }

    /** 
     * Runs the climber motors back to the hard stop and zeroes them
     */
    public Command zero() {
        return setVoltage(ClimbK.zeroingVoltage)
        .andThen(Commands.waitUntil(() -> homingDebouncer.calculate(talon.getSupplyCurrent().getValue().gte(ClimbK.homingSpike))))
        .andThen(() -> { 
            talon.setPosition(ClimbK.minAngle);
            isZeroed = true;
        })
        .finallyDo(this::stop)
        .withName("Zero");
    }

    /** Motion magic version of the Release command.
     */ 
    // public Command releaseMotionMagic() {
    //     MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.minAngle);
    //     return runOnce(() -> armMotor.setControl(request))
    //     .andThen(Commands.waitUntil(() -> armMotor.getPosition().getValue().isNear(ClimbK.minAngle, ClimbK.allowableError)))
    //     .withName("Release Motion Magic");
    // }

    /**
     * Stops the climb motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
        servo.set(0.5);
        servo.setAngle(ClimbK.servoInactiveAngle.in(Degree));
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

}
