package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;

public class Intake extends SubsystemBase {
    
    SparkMax motorLeader = new SparkMax(IntakeK.motorId, MotorType.kBrushless);
    SparkMax motorFollower = new SparkMax(IntakeK.motorId, MotorType.kBrushless);

    public Intake() {
        configSparkMax();
    }

    public void configSparkMax() {
        SparkMaxConfig motorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig motorFollowerConfig = new SparkMaxConfig();  // MAKE THIS NOT FOLLOW

        motorFollowerConfig.follow(motorLeader); //! Update / Finish
        motorLeaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); //! Update / Finish

        motorLeader.configure(motorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorFollower.configure(motorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command coralIntake(Voltage volts) {
        return runOnce(() -> motorLeader.setVoltage(volts));
    }

    public Command scoreLevelOne(Voltage volts) {
        return runOnce(() -> motorLeader.setVoltage(volts)); //MAKE THIS TAKE IN NEGATIVE VOLTS
    }

    public void scoreLevelTwoThree() {

    }

    public void scoreLevelFour() {

    }

    public void setSpeed() {

    }

    public void stop() {

    }

}
