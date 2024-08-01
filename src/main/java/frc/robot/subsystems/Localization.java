package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Localization {
    private final VisionSubsystem visionSubsystem;

    //use actual coordinates from apriltag library/normal java hashmap or cad
    private static final Pose2d[] APRILTAG_POSES = new Pose2d[] {
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        //more pose2d
    };

    public Localization(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    public Pose2d getRobotPose() {
        PhotonPipelineResult result = visionSubsystem.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0 && fiducialId < APRILTAG_POSES.length) {
                Pose2d tagPose = APRILTAG_POSES[fiducialId];
                return PhotonUtils.estimateFieldToRobotAprilTag(tagPose, target.getBestCameraToTarget());
            }
        }
        return null;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
}
