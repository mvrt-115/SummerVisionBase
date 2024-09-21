// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

  /** Creates a new Localizer. */
  public Localizer(Swerve swerve) {
    this.camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    
    this.cameraEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.camToRobot);

    this.swerve = swerve;
    this.poseEstimator = swerve.getPoseEstimator();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
