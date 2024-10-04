package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Localization {
    //use Field2D for logging (add that)
    private final VisionSubsystem visionSubsystem;

    //use actual coordinates from AprilTag library/normal Java hashmap or CAD
    private static final Pose3d[] APRILTAG_POSES = new Pose3d[] {
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
        //more poses
    };

    private static final Transform3d cameraToRobot = new Transform3d(
        new Translation3d(4 * 0.0254, 11 * 0.0254, 13 * 0.0254),
        new Rotation3d(0, 0, Math.PI)
    );

    private final Field2d field;

    public Localization(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.field = new Field2d();
    }

    public Pose2d getRobotPose() {
        PhotonPipelineResult result = visionSubsystem.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0 && fiducialId < APRILTAG_POSES.length) {
                Pose3d tagPose = APRILTAG_POSES[fiducialId];
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose, cameraToRobot);
                
                Pose2d robotPose2d = new Pose2d(
                robotPose3d.getTranslation().toTranslation2d(),
                new Rotation2d(robotPose3d.getRotation().getZ())
            );

            SmartDashboard.putNumber("Robot X (meters)", robotPose2d.getX());
            SmartDashboard.putNumber("Robot Y (meters)", robotPose2d.getY());
            SmartDashboard.putNumber("Robot Heading (radians)", robotPose2d.getRotation().getRadians());

            //field.setRobotPose(robotPose2d);

            return robotPose2d;
            }
        }
        return null;
    }   

    public void periodic() {
        CommandScheduler.getInstance().run();
    
        Pose2d robotPose = getRobotPose();
        if (robotPose != null) {
            SmartDashboard.putString("Robot Pose", robotPose.toString());
        } 
        else {
            SmartDashboard.putString("Robot Pose", "No targets detected");
        }
        SmartDashboard.putData("Field (test)", field);
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
}
