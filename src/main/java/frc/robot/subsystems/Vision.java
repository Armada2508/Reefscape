package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionK;
import frc.robot.Field;

public class Vision extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private final PhotonCamera frontCamera = new PhotonCamera(VisionK.frontCameraName);
    private final PhotonCamera backCamera = new PhotonCamera(VisionK.backCameraName);
    private final PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionK.robotToFrontCamera);
    private final PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionK.robotToBackCamera);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Vision");
    private final StructPublisher<Pose3d> pubFront = table.getStructTopic(VisionK.frontCameraName + " StdDevs/estimatedPose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> pubBack = table.getStructTopic(VisionK.backCameraName + " StdDevs/estimatedPose", Pose3d.struct).publish();
    private PhotonPipelineResult frontLatestResult;
    private PhotonPipelineResult backLatestResult;

    @Override
    public void periodic() {
        if (isCameraConnectedFront()) {
            frontLatestResult = frontCamera.getLatestResult();
        }
        if (isCameraConnectedBack()) {
            backLatestResult = backCamera.getLatestResult();
        }
    }

    /**
     * Returns a wrapper object containing a list of estimated robot poses and their standard deviations stored as a Pair to be added into 
     * a robot's odometry with PoseEstimator#addVisionMeasurement.
     */
    public VisionResults getVisionResults() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionResults = new ArrayList<>();
        if (isCameraConnectedFront()) {
            visionResults.addAll(processResults(frontCamera.getAllUnreadResults(), frontPoseEstimator, frontCamera.getName()));
        }
        if (isCameraConnectedBack()) {
            visionResults.addAll(processResults(backCamera.getAllUnreadResults(), backPoseEstimator, backCamera.getName()));
        }
        return new VisionResults(visionResults);
    }

    /**
     * Processes a list of photonvison results into a list of estimated poses and their respective standard deviations
     */
    private List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> processResults(List<PhotonPipelineResult> results, PhotonPoseEstimator poseEstimator, String name) {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionResults = new ArrayList<>();
        for (var result : results) {
            poseEstimator.update(result).ifPresent((pose) -> {
                if (isValidPose(pose)) {
                    var stdDevs = getStdDevs(result, pose, poseEstimator, name);
                    table.getEntry(name + " StdDevs/Standard Deviations").setDoubleArray(stdDevs.getData());
                    visionResults.add(Pair.of(pose, stdDevs));
                }
            });
        }
        return visionResults;
    }

    /**
     * Checks whether a pose is within the bounds of the field and the acceptable Z (height) error
     */
    private boolean isValidPose(EstimatedRobotPose pose) {
        var pose3d = pose.estimatedPose;
        return pose3d.getMeasureX().gte(Meters.zero()) 
            && pose3d.getMeasureX().lte(Field.fieldLength)
            && pose3d.getMeasureY().gte(Meters.zero())
            && pose3d.getMeasureY().lte(Field.fieldWidth)
            && pose3d.getMeasureZ().gte(VisionK.minPoseZ)
            && pose3d.getMeasureZ().lte(VisionK.maxPoseZ);
    }

    /**
     * Returns the standard deviations for a given photonvision result and estimated pose
     */
    private Matrix<N3, N1> getStdDevs(PhotonPipelineResult result, EstimatedRobotPose pose, PhotonPoseEstimator poseEstimator, String name) {
        int numTags = 0;
        double avgDistMeters = 0; 
        for (var target : result.getTargets()) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDistMeters += tagPose.get().toPose2d().getTranslation().getDistance(pose.estimatedPose.getTranslation().toTranslation2d());
        }
        avgDistMeters /= numTags;
        double stdevScalar = avgDistMeters / VisionK.baseLineAverageTagDistance.in(Meters);
        // Logging
        if (name == VisionK.frontCameraName) pubFront.accept(pose.estimatedPose);
        else pubBack.accept(pose.estimatedPose);
        table.getEntry(name + " StdDevs/numTags").setInteger(numTags);
        table.getEntry(name + " StdDevs/Average Distance to Tag (in.) RF").setDouble(Units.metersToInches(avgDistMeters)); // Robot Frame
        table.getEntry(name + " StdDevs/stdevScalar").setDouble(stdevScalar);
        if (numTags == 0) return VisionK.untrustedStdDevs;
        if (numTags == 1) {
            return VisionK.singleTagStdDevs.times(stdevScalar);
        }
        return VisionK.multiTagStdDevs.times(stdevScalar);
    }

    /**
     * Returns whether the front camera is connected
     */
    @Logged(name = "Front Camera Connected")
    public boolean isCameraConnectedFront() {
        return frontCamera.isConnected();
    } 

    /**
     * Returns whether the back camera is connected
     */
    @Logged(name = "Back Camera Connected")
    public boolean isCameraConnectedBack() {
        return backCamera.isConnected();
    } 

    /**
     * Returns whether the front camera can see an april tag
     */
    @Logged(name = "Front Camera Sees Tag")
    public boolean canSeeTagFront() {
        if (!isCameraConnectedFront()) return false;
        return frontLatestResult.hasTargets();    
    }

    /**
     * Returns whether the back camera can see an april tag
     */
    @Logged(name = "Back Camera Sees Tag")
    public boolean canSeeTagBack() {
        if (!isCameraConnectedBack()) return false;
        return backLatestResult.hasTargets();    
    }

    /**
     * Returns the ID of the best april tag seen by the front camera or -1 if no tag is seen
     */
    @Logged(name = "Front Camera Best Tag ID")
    public int bestTagIDFront() {
        if (!canSeeTagFront()) return -1;
        return frontLatestResult.getBestTarget().getFiducialId();
    }

    /**
     * Returns the ID of the best april tag seen by the back camera  or -1 if no tag is seen
     */
    @Logged(name = "Back Camera Best Tag ID")
    public int bestTagIDBack() {
        if (!canSeeTagBack()) return -1;
        return backLatestResult.getBestTarget().getFiducialId();
    }

    /**
     * Returns the number of april tags seen by the front camera
     */
    @Logged(name = "Front Camera Number Tags Seen")
    public int numTagsSeenFront() {
        if (!canSeeTagFront()) return 0;
        return frontLatestResult.getTargets().size();
    }

    /**
     * Returns the number of april tags seen by the back camera
     */
    @Logged(name = "Back Camera Number Tags Seen")
    public int numTagsSeenBack() {
        if (!canSeeTagBack()) return 0;
        return backLatestResult.getTargets().size();
    }

    /**
     * Returns the normal distance to the best tag in inches from the front camera (Camera Frame) or -1 if no tag is seen
     */
    @Logged(name = "Front Camera Normal Distance to Best Tag")
    public double distanceToBestTagFront() {
        if (!canSeeTagFront()) return -1;
        return Units.metersToInches(frontLatestResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
    }

    /**
     * Returns the normal distance to the best tag in inches from the back camera (Camera Frame) or -1 if no tag is seen
     */
    @Logged(name = "Back Camera Normal Distance to Best Tag")
    public double distanceToBestTagBack() {
        if (!canSeeTagBack()) return -1;
        return Units.metersToInches(backLatestResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
    }

    // This is your poor man's type alias, allows me to shorten the type and reference it by using VisionResults instead of List<Pair<EstimatedRobotPose, Matrix<N3, N1>>>
    public record VisionResults(List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> results){}

}
