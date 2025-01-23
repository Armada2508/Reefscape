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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionK;
import frc.robot.Field;

@Logged
public class Vision extends SubsystemBase {

    private final PhotonCamera frontCamera = new PhotonCamera(VisionK.frontCameraName);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionK.robotToCamera);

    public VisionResults getVisionResults() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionResults = new ArrayList<>();
        var results = frontCamera.getAllUnreadResults();
        for (var result : results) {
            poseEstimator.update(result).ifPresent((pose) -> {
                if (isValidPose(pose)) {
                    visionResults.add(Pair.of(pose, getStdDevs(result, pose)));
                }
            });
        }
        return new VisionResults(visionResults);
    }

    private boolean isValidPose(EstimatedRobotPose pose) {
        var pose3d = pose.estimatedPose;
        return pose3d.getMeasureX().gte(Meters.zero()) 
            && pose3d.getMeasureX().lte(Field.fieldLength)
            && pose3d.getMeasureY().gte(Meters.zero())
            && pose3d.getMeasureY().lte(Field.fieldWidth)
            && pose3d.getMeasureZ().gte(VisionK.minPoseZ)
            && pose3d.getMeasureZ().lte(VisionK.maxPoseZ);
    }

    // TODO: Implement this function
    private Matrix<N3, N1> getStdDevs(PhotonPipelineResult result, EstimatedRobotPose pose) {
        return VecBuilder.fill(0, 0, 0);
    }

    public boolean isCameraConnected() {
        return frontCamera.isConnected();
    } 

    public boolean canSeeTag() {
        if (!isCameraConnected()) return false;
        return frontCamera.getLatestResult().hasTargets();    
    }

    public int bestTagID() {
        if (!canSeeTag()) return -1;
        return frontCamera.getLatestResult().getBestTarget().getFiducialId();
    }

    public int numTagsSeen() {
        if (!canSeeTag()) return 0;
        return frontCamera.getLatestResult().getTargets().size();
    }

    /**
     * Returns the normal distance to the best tag in inches
     * @return
     */
    public double distanceToBestTag() {
        if (!canSeeTag()) return -1;
        return Units.metersToInches(frontCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
    }

    // This is your poor man's type alias
    public record VisionResults(List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> results){}

}
