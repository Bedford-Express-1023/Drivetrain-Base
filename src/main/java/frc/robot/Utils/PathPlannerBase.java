// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * @author Stephen Oz
 */
public class PathPlannerBase extends PPSwerveControllerCommand {
    public SwerveDriveSubsystem drivetrain;
    public PathPlannerTrajectory trajectory;

    private PathPlannerBase(PathPlannerTrajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
            PIDController xController, PIDController yController, ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates, Subsystem[] requirements) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements);
    }

    public PathPlannerBase(SwerveDriveSubsystem drivetrain, PathPlannerTrajectory trajectory) {
        super(
            trajectory, 
            drivetrain::getRealOdometry, 
            drivetrain.kinematics, 
            new EZEditPID(8, 0, 0.0, "PPXVel"), 
            new EZEditPID(8, 0, 0.0, "PPYVel"), 
            new EZEditProfiledPID(8, 0.0, 0.0, new TrapezoidProfile.Constraints(5.0,6.0), "PPRotation"), 
            (states) -> {
                    drivetrain.setStates(states);
            }, 
            drivetrain);
            this.drivetrain = drivetrain;
            this.trajectory = trajectory;
    }
    
    @Override
    public void initialize() {
        drivetrain.odometry.resetPosition(
            new Pose2d(
                trajectory.getInitialPose().getTranslation().plus(
                    new Translation2d(
                        -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2, 
                        -Constants.DRIVETRAIN_WHEELBASE_METERS / 2)),
                trajectory.getInitialPose().getRotation()), 
            drivetrain.getRotation());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeeds(new ChassisSpeeds(0, 0, 0));
        super.end(false);
    }
}
