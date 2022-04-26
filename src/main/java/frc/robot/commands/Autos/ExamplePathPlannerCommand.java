// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utils.PathPlannerBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExamplePathPlannerCommand extends SequentialCommandGroup {
/** Creates a new fiveBall. */

public ExamplePathPlannerCommand(SwerveDriveSubsystem drivetrain) {
  PathPlannerBase fourBall1 = new PathPlannerBase(drivetrain, PathPlanner.loadPath("4 ball 1", 3, 4));
  PathPlannerBase fourBall2 = new PathPlannerBase(drivetrain, PathPlanner.loadPath("4 ball 2", 3, 4));
  PathPlannerBase fourBall3 = new PathPlannerBase(drivetrain, PathPlanner.loadPath("4 ball 3", 3, 4));

  addCommands(
    fourBall1.deadlineWith(
      new WaitCommand(3)),
    fourBall2.deadlineWith(
      new WaitCommand(3)),
    fourBall3.deadlineWith(
      new WaitCommand(3))
  );
}
}
