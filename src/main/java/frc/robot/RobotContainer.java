package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Utils.EZEditPID;
import frc.robot.Utils.SendableDouble;
import frc.robot.commands.Autos.DoNothing;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.LimelightTarget;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.MovementTrackingTypes;

public class RobotContainer {
    private final SwerveDriveSubsystem m_drivetrain = new SwerveDriveSubsystem();

    public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    public final SendableDouble autoDelay = new SendableDouble("Auto Delay");

    private final LimelightTarget limelightTarget = new LimelightTarget(m_drivetrain);
    private final XboxController driverController = new XboxController(0);
    private final XboxController manipulatorController = new XboxController(1);

    private final SlewRateLimiter slewX = new SlewRateLimiter(6);
    private final SlewRateLimiter slewY = new SlewRateLimiter(6);


    
    public RobotContainer() {
        m_drivetrain.register();

        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("Option 1", new DoNothing());
        autoChooser.addOption("Option 2", new DoNothing());
        autoChooser.addOption("Option 3", new DoNothing());

        SmartDashboard.putData(autoChooser);
        SmartDashboard.putData(autoDelay);

        m_drivetrain.setDefaultCommand(new DriveCommand(
                m_drivetrain,
                () -> -modifyAxis(slewY.calculate(driverController.getLeftY())), // Axes are flipped here on purpose
                () -> -modifyAxis(slewX.calculate(driverController.getLeftX())),
                () -> -modifyAxis(driverController.getRightX()),
                () -> driverController.getLeftBumper(), //RobotCentric
                () -> (driverController.getRightBumper() ? 0.4 : 1.0), //lowPower
                () -> driverController.getRightTriggerAxis() > 0.5
        ));

        new Button(driverController::getBButtonPressed)
                .whenPressed(m_drivetrain::zeroGyroscope);
        new Button(() -> driverController.getLeftTriggerAxis() > 0.5)
                .whileHeld(() -> m_drivetrain.limelightTarget(true, MovementTrackingTypes.robotSpeed));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
    
}