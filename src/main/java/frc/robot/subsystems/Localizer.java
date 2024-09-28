// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private double count;

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

    System.out.println("in periodic!!");

      // This method will be called once per scheduler run
      var result = camera.getLatestResult();

      //Estimated position based on vision (camera) alone
      Optional<EstimatedRobotPose> estimatedPoseVision = cameraEstimator.update();

      //Estimated position based on swerve kinematics
      Rotation2d rot = new Rotation2d(swerve.getYaw());
      if (rot != null && swerve.getSwerveModulePositions() != null){
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rot, swerve.getSwerveModulePositions());
      }
      
      //Add vision measurement to poseEstimator (combining vision & swerve)
      //Uses position (pose2d), and timestamp of snapshot taken
      System.out.println(result.getTargets().size());
      count += 0.000001;
      SmartDashboard.putNumber("Vision - Value", count);
      if(result.getTargets().size()>0){
        SmartDashboard.putBoolean("Vision - Has Targets", true);
        poseEstimator.addVisionMeasurement(estimatedPoseVision.get().estimatedPose.toPose2d(), estimatedPoseVision.get().timestampSeconds);
      
      } else {
        SmartDashboard.putBoolean("Vision - Has Targets", false);
      }

      Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
      
      //Log this estimated position
      field.setRobotPose(estimatedPos);
      SmartDashboard.putData("Vision - Field", field); //Switch to advantagekit later
  }
}