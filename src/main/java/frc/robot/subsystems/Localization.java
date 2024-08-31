// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Localization extends SubsystemBase {
  /** Creates a new Localization. */

  // initialize camera
  private PhotonCamera camera = new PhotonCamera("photonvision");
  
  // placeholder robot/camera location values
  final double cameraHeight = Units.inchesToMeters(24);
  final double targetHeight = Units.inchesToMeters(5);
  final double cameraPitch = Units.degreesToRadians(0);
  final double goalMargin = Units.feetToMeters(2); // desired distance from goal

  // placeholder PID values

  private final double LINEAR_P = 0.0;
  private final double LINEAR_D = 0.0;

  private final double ANGULAR_P = 0.0;
  private final double ANGULAR_D = 0.0;

  private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  private PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  public Localization() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveToTarget() {
    
    double rotationSpeed;
    double moveSpeed;
    
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch, Units.degreesToRadians(result.getBestTarget().getPitch()));
      moveSpeed = -forwardController.calculate(range, goalMargin);
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      
    }
    else {
      rotationSpeed = 0;
      moveSpeed = 0;
    }
  }

  public Transform3d getRobotPosition() {
    Transform3d fieldToCamera = null;
    var result = camera.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      fieldToCamera = result.getMultiTagResult().estimatedPose.best;
    }
    return fieldToCamera;

  }

  

}
