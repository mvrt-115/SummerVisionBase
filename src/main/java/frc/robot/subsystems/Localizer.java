// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Localizer extends SubsystemBase {
  //The camera
  private PhotonCamera camera;
  //Pose estimation based on april tags ONLY, using specified pose strat
  private PhotonPoseEstimator cameraEstimator;
  //Pose estimation based on april tags AND odometry
  private SwerveDrivePoseEstimator poseEstimator;
  //Swerve
  private Swerve swerve;
  //Field layout (tag IDs on a map)
  private AprilTagFieldLayout fieldLayout;

  //"Field" for logging
  private Field2d field;

  /** Creates a new Localizer. */
  public Localizer(Swerve swerve) {
    this.camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    
    this.field = new Field2d();

    this.cameraEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.VisionConstants.camToRobot);

    this.swerve = swerve;
    this.poseEstimator = swerve.getPoseEstimator();
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      var result = camera.getLatestResult();

      //Estimated position based on vision (camera) alone
      Optional<EstimatedRobotPose> estimatedPoseVision = cameraEstimator.update();

      //Estimated position based on swerve kinematics
      Rotation2d rot = new Rotation2d(swerve.getYaw());
      SwerveModulePosition[] module_pos = swerve.getSwerveModulePositions();

      //TODO: Add null try-catch here (? issue)
      //Updating pose estimation with odometry & angle
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rot, module_pos);

      try {
        if(result.hasTargets() && estimatedPoseVision.isPresent()){
          SmartDashboard.putBoolean("Pingu - Has Targets", true);
          poseEstimator.addVisionMeasurement(estimatedPoseVision.get().estimatedPose.toPose2d(), result.getTimestampSeconds());
          
        } else{
          SmartDashboard.putBoolean("Pingu - Has Targets", false);
        }
      } catch (ConcurrentModificationException e){

      }

      /**
       * 
      if(Math.abs(lastTimeSinceVisionUpdate - Timer.getFPGATimestamp()) > 0.5){
        SmartDashboard.putBoolean("Pingu - Running vision update", true);
        if(result.hasTargets() && estimatedPoseVision.isPresent()){
          SmartDashboard.putBoolean("Pingu - Has Targets", true);
          poseEstimator.addVisionMeasurement(estimatedPoseVision.get().estimatedPose.toPose2d(), result.getTimestampSeconds());
          lastTimeSinceVisionUpdate = Timer.getFPGATimestamp();

        } else{
          SmartDashboard.putBoolean("Pingu - Has Targets", false);
        }
      }
       */
      
      Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
      
      //Log this estimated position
      field.setRobotPose(estimatedPos);
      SmartDashboard.putData("Pingu - Field", field); //Switch to advantagekit later
  }

  /**
   * @return current pose according to pose estimator
   */
  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return field layout (for Align command)
   */
  public AprilTagFieldLayout getFieldLayout(){
    return fieldLayout;
  }

  /**
   * @return location to use for alignment
   */
  public Pose2d getAlignLoc(){
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(7); //ID 4 for red speaker, ID 7 for blue speaker
    return tagPose.get().toPose2d();
  }
}