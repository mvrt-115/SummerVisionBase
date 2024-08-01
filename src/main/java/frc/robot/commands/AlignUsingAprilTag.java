package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

public class AlignUsingAprilTag extends Command {
    private final Localization localization;

    public AlignUsingAprilTag(Localization localization) {
        this.localization = localization;
        addRequirements(localization.getVisionSubsystem());
    }

    @Override
    public void execute() {
        Pose2d robotPose = localization.getRobotPose();
        if (robotPose != null) {
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
