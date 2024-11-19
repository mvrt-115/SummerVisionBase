// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ConcurrentModificationException;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  //TODO: add STDs

  //"Field" for logging
  private Field2d field;

  //For Align
  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;
  //private Supplier<Pose2d> poseSupplier;

  /** Creates a new Localizer. */
  public Localizer(Swerve swerve) {
    this.camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    
    this.field = new Field2d();

    this.cameraEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.VisionConstants.camToRobot);

    this.swerve = swerve;
    this.poseEstimator = swerve.getPoseEstimator();

    //For Align
    pidX = new PIDController(2.7, 0, 0.0);
    pidY = new PIDController(2.7, 0, 0.0);
    pidTheta = new PIDController(4, 0, 0);
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
      
      Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
      
      //Log this estimated position
      field.setRobotPose(estimatedPos);
      SmartDashboard.putData("Pingu - Field", field); //Switch to advantagekit later
  }

  /**
   * Return command to run alignment
   * @return
   */
   public RobotCentric getAlignCommand(){
        Pose2d robotPose = getCurrentPose();
        Pose2d targetPose = getAlignLoc();
        double theta = robotPose.getRotation().getRadians();

        SmartDashboard.putBoolean("Pingu ALIGN - Aligning", true);

        SmartDashboard.putNumber("Pingu ALING - Robot Pose X", robotPose.getX());
        SmartDashboard.putNumber("Pingu ALIGN - Robot Pose Y", robotPose.getY());

        SmartDashboard.putNumber("Pingu ALIGN - Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Pingu ALIGN - Target Pose Y", targetPose.getY());

        SmartDashboard.putNumber("Pingu ALIGN - Error X", Math.abs(robotPose.getX() - targetPose.getX()));
        SmartDashboard.putNumber("Pingu ALIGN - Error Y", Math.abs(robotPose.getY() - targetPose.getY()));

        double outX = pidX.calculate(robotPose.getX(), targetPose.getX()); // pos, setpoint
        double outY = pidY.calculate(robotPose.getY(), targetPose.getY());
        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(outX, outY, 0, new Rotation2d(-theta));

        SmartDashboard.putNumber("Ping ALIGN Z - SPEEDS", speeds.vxMetersPerSecond);

        //SmartDashboard.putNumber("Ping ALIGN X - SPEEDS", speeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("Ping ALIGN Y - SPEEDS", speeds.vyMetersPerSecond);
        //SmartDashboard.putNumber("Ping ALIGN Theta - SPEEDS", speeds.omegaRadiansPerSecond);

        return swerve.getRobotOriented().withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
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