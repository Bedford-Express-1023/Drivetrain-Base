package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.MovementTrackingTypes;

public class LimelightTarget extends CommandBase{
    SwerveDriveSubsystem drivetrain;
    public LimelightTarget(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.limelightTarget(false, MovementTrackingTypes.none);
    }
}
