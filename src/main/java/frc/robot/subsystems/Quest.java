package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestK;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
    QuestNav questNav = new QuestNav();
    
    private Swerve swerve = new Swerve(null, null);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
    
    private void GetPose() {
        // First, Declare our geometrical transform from the robot center to the Quest
        Transform2d ROBOT_TO_QUEST = new Transform2d( /*TODO: Put your x, y, rotational offsets here!*/ );

        // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
                // Get the most recent Quest pose
                Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();

                // Transform by the mount pose to get your robot pose
                Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        }
    }
    
    private void SetPose() {
        // First, Declare our geometrical transform from the robot center to the quest
        Transform2d ROBOT_TO_QUEST = new Transform2d( /*TODO: Put your x, y, rotational offsets here!*/ );

        // Assume this is the requested reset pose
        Pose2d robotPose = new Pose2d( /* Some pose data */ );

        // Transform by the offset to get the Quest pose
        Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

        // Send the reset operation
        questNav.setPose(questPose);
    }

        Matrix<N3, N1> QUESTNAV_STD_DEVS =
            VecBuilder.fill(
                0.02, // Trust down to 2cm in X direction
                0.02, // Trust down to 2cm in Y direction
                0.035 // Trust down to 2 degrees rotational
        );

    @Override
    public void periodic() {
        if (questNav.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                Pose2d questPose = questFrame.questPose();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose2d robotPose = questPose.transformBy(QuestK.questOffset.inverse());

                // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
                double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);

                // // You can put some sort of filtering here if you would like!

                // // Add the measurement to our estimator
                swerve.addVisionMeasurement(robotPose, timestamp, QUESTNAV_STD_DEVS);
            };
        }
}
    /*
     Attempt to make a QuestNav equivalent of VisionResults in Vision code
     Goal is to return a wrapper object containing estimated robot poses and their standard deviations
     */
    public QuestResults getQuestResults() {
        //List<PoseFrame> questResults = new ArrayList<>(); Remove later if code works
        PoseFrame[] questFrames = null;
        if (questNav.isConnected() && questNav.isTracking()) {
            questFrames = questNav.getAllUnreadPoseFrames();
            //questResults.addAll(questNav.getAllUnreadPoseFrames(), poseEstimator, questNav);  Remove later if code works
        }
        return new QuestResults(questFrames);
    }

    public record QuestResults(PoseFrame[] results){}
};



