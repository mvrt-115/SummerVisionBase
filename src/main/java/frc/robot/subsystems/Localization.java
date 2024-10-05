// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Localization extends SubsystemBase {
  /** Creates a new Localization. */

  /* Create a camera */
  private PhotonCamera camera;
  
  /* Temporary PID things for moving toward a target  
  private PIDController turnController = 
      new PIDController(Constants.CameraConstants.ANGULAR_P, 0, Constants.CameraConstants.ANGULAR_D);
  private PIDController forwardController = 
      new PIDController(Constants.CameraConstants.LINEAR_P, 0, Constants.CameraConstants.LINEAR_D);
  */

  /* April tag information */
  private AprilTagFieldLayout aprilTagFieldLayout;

  private PhotonPoseEstimator photonPoseEstimator; 
  private SwerveDrivePoseEstimator robotPoseEstimator;

  private Swerve swerve;

  private Field2d field;
  
  
  /*final Pose3d[] APRILTAG_POSITIONS = new Pose3d[] {
    new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
    // other apriltag positions                            
  }; */
  
  public Localization(Swerve swerve) {
    camera = new PhotonCamera("photonvision");
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                              Constants.CameraConstants.robotToCamera);
    
    this.swerve = swerve;
    robotPoseEstimator = swerve.getPoseEstimator();

    this.field = new Field2d();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("periodic");

    robotPoseEstimator.addVisionMeasurement(getEstimatedMultitagFieldPosition().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());

    Rotation2d rotation = new Rotation2d(swerve.getYaw());
      if (rotation != null && swerve.getSwerveModulePositions() != null){
        robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, swerve.getSwerveModulePositions());
      }
    SmartDashboard.putNumber("best target id", getBestTargetId());
    Pose2d estimatedPos = robotPoseEstimator.getEstimatedPosition();
    
    photonPoseEstimator= new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.CameraConstants.robotToCamera);

    field.setRobotPose(estimatedPos);
    SmartDashboard.putData("field-position", field);
  }

  /* 
  public void moveToTarget() {
    // placeholder variables for movement
    double rotationSpeed;
    double moveSpeed;
    
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      Constants.CameraConstants.cameraHeight, 
                      Constants.CameraConstants.targetHeight,
                      Constants.CameraConstants.cameraPitch, 
                      Units.degreesToRadians(result.getBestTarget().getPitch()));
      moveSpeed = -forwardController.calculate(range, Constants.CameraConstants.goalMargin);
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      
    }
    else {
      rotationSpeed = 0;
      moveSpeed = 0;
    }
  }
  */

  /**
   * Get transform3d to target
   * @return  Transform3d of camera to target
   */
  public Transform3d getCameraTransformToBestTarget() {
    var result = camera.getLatestResult();
    if  (result.hasTargets()) {
      var target = result.getBestTarget();
      int tagId = target.getFiducialId();
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      return cameraToTarget;
    }
    return null;
  }

  public int getBestTargetId() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      return target.getFiducialId();
    }
    return -1;
  }

  public Transform2d getRobotToTarget() {
    // incorporate distance of camera to drive base
    return null;
  }

  // have to update this to DifferentialDrivePoseEstimator using addVisionMeasurement()

  /**
   * photon pose estimator to be used in adding to robot pose estimate
   * @return updated photonPoseEstimator
   */
  public Optional<EstimatedRobotPose> getEstimatedMultitagFieldPosition() {
    return photonPoseEstimator.update();
  }

  /*
  public Pose2d getFieldPosition() {
    // get the position based on stuff you see and array of apriltag positions
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      int targetId = target.getFiducialId();
      Pose3d tagPosition = APRILTAG_POSITIONS[targetId];
      Transform3d cameraToTarget = target.getBestCameraToTarget();
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPosition, Constants.CameraConstants.robotToCamera);
                return new Pose2d(
                    robotPose3d.getTranslation().toTranslation2d(),
                    new Rotation2d(robotPose3d.getRotation().getZ())
                );
    }
    return null;
  }
  */

}
