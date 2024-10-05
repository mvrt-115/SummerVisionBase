// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Localization;

public class GetFieldCoordinates extends Command {

  private Localization localization;
  private double lastLoggedTime;

  private final int DEBUG_INTERVAL;

  /** Creates a new GetFieldCoordinates. */
  public GetFieldCoordinates(Localization localization) {
    // Use addRequirements() here to declare subsystem dependencies.
    lastLoggedTime = 0;
    DEBUG_INTERVAL = 10;

    this.localization = localization;
    
    addRequirements(localization);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Pose2d currentPose = localization.getFieldPosition();
    Transform3d poseToNearestTarget = localization.getCameraTransformToBestTarget();
    
    // logging pose to smart dashboard every once in a while yay yay yippee
    if (Timer.getFPGATimestamp() - lastLoggedTime >= DEBUG_INTERVAL) {
      // translating the pose to an array of doubles with x, y, and rotation so it can be logged
      //double[] poseArray = {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()};
      //SmartDashboard.putNumberArray("position at " + Timer.getFPGATimestamp(), poseArray);
      // translating the x, y, z from the transform3d distance from the target to be logged
      double[] distanceArray = {poseToNearestTarget.getX(), poseToNearestTarget.getY(), poseToNearestTarget.getZ()};
      SmartDashboard.putNumberArray("transform to target at " + Timer.getFPGATimestamp(), distanceArray);
      //SmartDashboard.putNumber("target id", localization.getBestTargetId());

      lastLoggedTime = Timer.getFPGATimestamp();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
