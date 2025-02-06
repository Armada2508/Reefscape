package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDK;

public class LED extends SubsystemBase {
    private AddressableLED driveBaseLED = new AddressableLED(LEDK.driveBaseLEDPort);
    private AddressableLED elevatorLED = new AddressableLED(LEDK.elevatorLEDPort);
    private AddressableLEDBuffer driveBaseLEDBuffer = new AddressableLEDBuffer(LEDK.driveBaseLEDLength);
    private AddressableLEDBuffer elevatorLEDBuffer = new AddressableLEDBuffer(LEDK.elevatorLEDLength);
    AddressableLEDBufferView driveBaseStrip = driveBaseLEDBuffer.createView(0, LEDK.driveBaseLEDLength);
    AddressableLEDBufferView elevatorStrip = elevatorLEDBuffer.createView(LEDK.driveBaseLEDLength + 1, LEDK.driveBaseLEDLength + LEDK.elevatorLEDLength + 1);

    public LED() {
        driveBaseLED.setLength(driveBaseLEDBuffer.getLength());
        elevatorLED.setLength(elevatorLEDBuffer.getLength());

        driveBaseLED.setData(driveBaseLEDBuffer);
        elevatorLED.setData(elevatorLEDBuffer);

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