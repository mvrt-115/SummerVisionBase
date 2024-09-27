package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignUsingAprilTag extends Command {
    private final Localization localization;

    public AlignUsingAprilTag(Localization localization) {
        this.localization = localization;
        addRequirements(localization.getVisionSubsystem());
    }

    public void logAlignmentData(String data) {
        //log alignment data to SmartDashboard
        SmartDashboard.putString("Aligning to AprilTag", data);
    }

    @Override
    public void execute() {
        Pose2d robotPose = localization.getRobotPose();
        if (robotPose != null) {
            logAlignmentData(robotPose.toString());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
