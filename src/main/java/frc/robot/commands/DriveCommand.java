package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final SwerveDriveSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    //public final DoubleSupplier rotationXSupplier;
    //public final DoubleSupplier rotationYSupplier;
    private final BooleanSupplier robotCentric;
    public DoubleSupplier drivePower;
    public DoubleSupplier TurnSpeed;
    public DoubleSupplier rotationSupplier;

    public DriveCommand(
            SwerveDriveSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier robotCentric,
            DoubleSupplier drivePower
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentric = robotCentric;
        this.drivePower = drivePower;
        addRequirements(drivetrain);
    }
    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        double drivePower = this.drivePower.getAsDouble();
        if (robotCentric.getAsBoolean()) {
            drivetrain.setSpeeds(
                new ChassisSpeeds(
                        translationXPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                        rotationPercent * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                    )
            );
        } else {
            drivetrain.setSpeeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationXPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            translationYPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            rotationPercent * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            drivetrain.getRotation()
                )
            );
        }
    }
    //for special right joystick control
    /*@Override
    public void execute() {
        drivetrain.CommandVariable = "Drive";
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationTarget = Math.PI + 2*Math.PI - Math.atan2(rotationYSupplier.getAsDouble(), rotationXSupplier.getAsDouble());
        if (this.lowPower.getAsBoolean()) {drivePower = 0.2;} 
        if (robotCentric.getAsBoolean()) {
            drivetrain.drive(
                new ChassisSpeeds(
                        translationXPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                        (Math.hypot(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble()) > 0.9) ? drivetrain.rotationPID.calculate(drivetrain.getRotation().getRadians(), rotationTarget-Math.PI/2) : 0
                    )
            );
        } else {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationXPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            translationYPercent * drivePower * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            (Math.hypot(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble()) > 0.9) ? drivetrain.rotationPID.calculate(drivetrain.getRotation().getRadians(), rotationTarget-Math.PI/2) : 0,
                            drivetrain.getRotation()
                )
            );
        }
        this.drivePower = 1.0;
    }*/

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}