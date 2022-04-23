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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class PathPlannerBase extends PPSwerveControllerCommand {
    public SwerveDriveSubsystem drivetrain;

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
            new PIDController(8, 0, 0.0), 
            new PIDController(8, 0.0, 0.0), 
            new ProfiledPIDController(8, 0.0, 0.0, new TrapezoidProfile.Constraints(5.0,6.0)), 
            (states) -> {
                    drivetrain.setStates(states);
            }, 
            drivetrain);
            this.drivetrain = drivetrain;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeeds(new ChassisSpeeds(0, 0, 0));
        super.end(false);
    }
}
