package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointTowardsHub extends CommandBase {
    private final SwerveDriveSubsystem drivetrain;
    public PIDController LimeLightRotationPID;


    public PointTowardsHub(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.CommandVariable = "TowardsHub";
        drivetrain.pointTowardsHub();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}