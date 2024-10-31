// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Swerve;

public class Align extends Command {

  private Swerve swerve;
  private Localizer localizer;
  Supplier<Pose2d> poseSupplier;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidTheta;

  private AprilTagFieldLayout fieldLayout;

  private Pose2d targetPose;

  /** Creates a new Align. */
  public Align(Swerve swerve, Localizer localizer, Supplier<Pose2d> poseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.localizer = localizer;
    this.poseSupplier = poseSupplier;

    pidX = new PIDController(2.7, 0, 0.0);
    pidY = new PIDController(2.7, 0, 0.0);
    pidTheta = new PIDController(4, 0, 0);

    fieldLayout = localizer.getFieldLayout();

    addRequirements(this.swerve, this.localizer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = poseSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = localizer.getCurrentPose();
    double theta = robotPose.getRotation().getRadians();

    double outX = pidX.calculate(robotPose.getX(), targetPose.getX()); // pos, setpoint
    double outY = pidY.calculate(robotPose.getY(), targetPose.getY());
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(outX, outY, 0, new Rotation2d(-theta));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d robotPose = localizer.getCurrentPose();

    double xTolerance = 0.05;
    double yTolerance = 0.03;

    return Math.abs(robotPose.getX() - targetPose.getX()) < xTolerance &&
      Math.abs(robotPose.getY() - targetPose.getY()) < yTolerance;
  }
}
