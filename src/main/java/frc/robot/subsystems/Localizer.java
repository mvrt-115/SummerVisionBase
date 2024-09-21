// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Localizer extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator cameraEstimator; //possibly wrong to use at all

  private SwerveDrivePoseEstimator poseEstimator;
  private Swerve swerve;

  private AprilTagFieldLayout fieldLayout;

  /** Creates a new Localizer. */
  public Localizer() {
    this.camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
