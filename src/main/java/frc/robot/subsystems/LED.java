package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDK;

public class LED extends SubsystemBase {
    // [Drive Base]-[Elevator Left]-[Elevator Right]-[Climb]
    private AddressableLED driveBaseLED = new AddressableLED(LEDK.driveBaseLEDPort);
    private AddressableLED leftElevatorLED = new AddressableLED(LEDK.leftElevatorLEDPort);
    private AddressableLED rightElevatorLED = new AddressableLED(LEDK.rightElevatorLEDPort);
    private AddressableLED climbLED = new AddressableLED(LEDK.climbLEDPort);

    private AddressableLEDBuffer driveBaseLEDBuffer = new AddressableLEDBuffer(LEDK.driveBaseLEDLength);
    private AddressableLEDBuffer leftElevatorLEDBuffer = new AddressableLEDBuffer(LEDK.elevatorLEDLength);
    private AddressableLEDBuffer rightElevatorLEDBuffer = new AddressableLEDBuffer(LEDK.elevatorLEDLength);
    private AddressableLEDBuffer climbLEDBuffer = new AddressableLEDBuffer(LEDK.climbLEDLength);

    //^ In theory we need these but I dont see any implementation of them
    // private AddressableLEDBufferView driveBaseStrip = driveBaseLEDBuffer.createView(0, LEDK.driveBaseLEDLength);
    // private AddressableLEDBufferView leftElevatorStrip = leftElevatorLEDBuffer.createView(LEDK.driveBaseLEDLength + 1, LEDK.driveBaseLEDLength + LEDK.elevatorLEDLength + 1);
    // private AddressableLEDBufferView rightElevatorStrip = rightElevatorLEDBuffer.createView(0, 0);
    // private AddressableLEDBufferView climbStrip = climbLEDBuffer.createView(0, 0);


    public LED() {
        driveBaseLED.setLength(driveBaseLEDBuffer.getLength());
        leftElevatorLED.setLength(leftElevatorLEDBuffer.getLength());
        rightElevatorLED.setLength(rightElevatorLEDBuffer.getLength());
        climbLED.setLength(climbLEDBuffer.getLength());

        driveBaseLED.start();
        leftElevatorLED.start();
        rightElevatorLED.start();
        climbLED.start();

        setDefaultCommand(run(() -> LEDPattern.solid(Color.kBlack)).withName("Off"));
    }

    @Override
    public void periodic() {
        driveBaseLED.setData(driveBaseLEDBuffer);
        leftElevatorLED.setData(leftElevatorLEDBuffer);
        rightElevatorLED.setData(rightElevatorLEDBuffer);
        climbLED.setData(climbLEDBuffer);
    }

    /**
     * 
     * @return 
     */
    public static LEDPattern robotStatusPattern() {
        return null;
    }

    /**
     * 
     * @return
     */
    public static LEDPattern coralStatusPattern() {
        return null;
    }

    /**
     * 
     * @return
     */
    public static LEDPattern climbStatusPattern() {
        return null;
    }
}


/*
1. Robot Status
	- Shines a different color depending on what mode the robot is in, also time in match (Auto, Teleop, last 15)
2. Coral Status
	- Shows current status of coral within the robot
3. Climb Status
	- Shows current status of robot climb
4. Action Animations
	- Shows what the robot is doing, what routine is being called
*/