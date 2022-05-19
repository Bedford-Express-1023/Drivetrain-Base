package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveLib.SwerveModule;
import frc.robot.Utils.EZEditPID;
import frc.robot.Utils.EZEditProfiledPID;
import frc.robot.Utils.SendableDouble;
import frc.robot.SwerveLib.Mk4SwerveModuleHelper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
        public final SwerveModule frontLeftModule;
        public final SwerveModule frontRightModule;
        public final SwerveModule backLeftModule;
        public final SwerveModule backRightModule;
        public ChassisSpeeds previousSpeeds = new ChassisSpeeds(0,0,0);
        public String CommandVariable = "None";
        public SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

        public final Pigeon2 gyroscope = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);
        public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.45, 1.85, 0.000037994);
        public final EZEditPID[] velocityPID = {new EZEditPID(0.25, 0.0, 0.0, Shuffleboard.getTab("PID"), "FLVelocity"), new EZEditPID(0.25, 0.0, 0.0, Shuffleboard.getTab("Drivetrain"), "FRVelocity"), new EZEditPID(0.25, 0.0, 0.0, Shuffleboard.getTab("Drivetrain"), "BLVelocity"), new EZEditPID(0.25, 0.0, 0.0, Shuffleboard.getTab("Drivetrain"), "BRVelocity")};
        public final EZEditProfiledPID pidRot = new EZEditProfiledPID(4.5, 0.0, 0.005, new TrapezoidProfile.Constraints(100, 5), Shuffleboard.getTab("PID"), "FLVelocity");
        public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
        public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroscope.getCompassHeading()));

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        public boolean RobotCentric = false;

        public SwerveDriveSubsystem() {
                pidRot.enableContinuousInput(0, 2*Math.PI);
                ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

                frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(1, 4)
                                .withPosition(0, 0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                        Constants.FRONT_LEFT_MODULE_STEER_OFFSET
                );

                frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                .withSize(1, 4)
                                .withPosition(1, 0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
                );

                backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                .withSize(1, 4)
                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                        Constants.BACK_LEFT_MODULE_STEER_OFFSET
                );

                backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                .withSize(1, 4)
                                .withPosition(3, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                        Constants.BACK_RIGHT_MODULE_STEER_OFFSET
                );
                ShuffleboardLayout orientationLayout = shuffleboardTab.getLayout("Orientation", BuiltInLayouts.kList).withSize(1, 3).withPosition(4,0);
                orientationLayout.addNumber("Pose X", () -> getRealOdometry().getX());
                orientationLayout.addNumber("Pose Y", () -> getRealOdometry().getY());
                orientationLayout.addNumber("Rotation", () -> getRotation().getDegrees());

        
                ShuffleboardLayout PigeonInfo = shuffleboardTab.getLayout("Pigeon Gyroscope", BuiltInLayouts.kList).withPosition(5, 0).withSize(1, 3);
                PigeonInfo.addNumber("Yaw", () -> gyroscope.getYaw());
                PigeonInfo.addNumber("Pitch", () -> gyroscope.getPitch());
                PigeonInfo.addNumber("Roll", () -> gyroscope.getRoll());
                shuffleboardTab.addBoolean("Robot Centric", () -> RobotCentric);
        }   
        public void zeroGyroscope() {
                odometry.resetPosition(
                        new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                        Rotation2d.fromDegrees(gyroscope.getYaw())
                );
        }

        public Rotation2d getRotation() {
                return odometry.getPoseMeters().getRotation();
        }
    
        public void setStates(SwerveModuleState... states ) {
                this.chassisSpeeds = kinematics.toChassisSpeeds(states);
                this.states = states;
        }

        public void setSpeeds(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
                this.states = kinematics.toSwerveModuleStates(chassisSpeeds);
        }
    
        //allows setting specific values and leaving others unchanged
        public void setSpeeds(Optional<Double> vxMetersPerSecond, Optional<Double> vyMetersPerSecond, Optional<Double> omegaRadiansPerSecond) {
                ChassisSpeeds newSpeeds = this.chassisSpeeds;
                vxMetersPerSecond.ifPresent((x) -> newSpeeds.vxMetersPerSecond = x);
                vyMetersPerSecond.ifPresent((y) -> newSpeeds.vyMetersPerSecond = y);
                omegaRadiansPerSecond.ifPresent((o) -> newSpeeds.omegaRadiansPerSecond = o);
                this.chassisSpeeds = newSpeeds;
                this.states = kinematics.toSwerveModuleStates(newSpeeds);
        }
        /**
        * Rotates the reference frame around the robot
        * @param referenceFrameRotation the absolute rotation of the reference frame, 0 being left positive Y and forward positive X, positve values rotate counter-clockwise
        * @return the speed of the robot in a rotated reference frame
        */
        public Translation2d getRotatedFrame(Rotation2d referenceFrameRotation) {
                return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(getRotation().minus(referenceFrameRotation));
        }

        public void updateOdometry() {
                odometry.update(Rotation2d.fromDegrees(gyroscope.getYaw()),
                        new SwerveModuleState(frontLeftModule.getDriveVelocity()*2.025, new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModuleState(frontRightModule.getDriveVelocity()*2.025, new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModuleState(backLeftModule.getDriveVelocity()*2.025, new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModuleState(backRightModule.getDriveVelocity()*2.025, new Rotation2d(backRightModule.getSteerAngle()))
                );
        }

        /**
        * Gets the real odometry of the robot 
        * @return the position and rotation of the robot focused on its center
        */
        public Pose2d getRealOdometry() {
                return new Pose2d(
                                odometry.getPoseMeters().getTranslation().plus(new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2, -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2).rotateBy(getRotation())),
                                //odometry.getPoseMeters().getX() - centerToWheel*Math.cos(Math.toDegrees(-135 + 90) + getRotation().getRadians()),
                                //odometry.getPoseMeters().getY() - centerToWheel*Math.sin(Math.toDegrees(-135 + 90) + getRotation().getRadians()),
                                getRotation()
                );
        }

        public enum MovementTrackingTypes{
                robotSpeed, targetMovement, none
        }

        double previousRotation;
        double previousLimelight;
        /**
         * Uses the swerve to track a limelight target
         * @param SWiM if the targetting should take into account movement of either the robot or the target
         * @param movementTrackingType 
         */
        public void limelightTarget(Boolean SWiM, MovementTrackingTypes movementTrackingType) {
                double fudge = 4; //TODO: tune fudge1 based on ball flight speed
                //finds the speed of the robot relative to the target
                NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
                Translation2d targetRelativeSpeeds = getRotatedFrame(getRotation().plus(Rotation2d.fromDegrees(180 + -limelight.getEntry("tx").getDouble(0))));
                double toAddRadians = 0;
                SmartDashboard.putNumber("TargetRelativeX", targetRelativeSpeeds.getX());
                SmartDashboard.putNumber("TargetRelativeY", targetRelativeSpeeds.getY());

                if (movementTrackingType == MovementTrackingTypes.robotSpeed && SWiM) {
                        toAddRadians = -Math.PI/2 +Math.acos(targetRelativeSpeeds.getY() / fudge);
                }
                
                if (movementTrackingType == MovementTrackingTypes.targetMovement && SWiM) {
                        toAddRadians = Math.toRadians(limelight.getEntry("tx").getDouble(0) - previousLimelight) * fudge + previousRotation;
                }

                double toRotate = pidRot.calculate(0, toAddRadians + (-limelight.getEntry("tx").getDouble(0) * Math.PI/180));
                if (limelight.getEntry("tv").getDouble(0) == 1) {
                        previousRotation = toRotate;
                        previousLimelight = limelight.getEntry("tx").getDouble(0);
                        setSpeeds(Optional.ofNullable(null), Optional.ofNullable(null), Optional.ofNullable(toRotate + toAddRadians));
                }
                if (limelight.getEntry("tx").getDouble(0) > -3 && limelight.getEntry("tx").getDouble(0) < 3 ) {
                        setSpeeds(Optional.ofNullable(null), Optional.ofNullable(null), Optional.ofNullable(0.0));  
                }
        }

        @Override
        public void periodic() {
                //SmartDashboard.putData(new SendableDouble(CommandVariable))
                updateOdometry();
                frontLeftModule.set((feedForward.calculate(states[0].speedMetersPerSecond) + velocityPID[0].calculate(frontLeftModule.getDriveVelocity(), states[0].speedMetersPerSecond)) * Math.min(Constants.SWERVE_SPEED_MULTIPLIER, 1), states[0].angle.getRadians());
                frontRightModule.set((feedForward.calculate(states[1].speedMetersPerSecond) + velocityPID[1].calculate(frontRightModule.getDriveVelocity(), states[1].speedMetersPerSecond)) * Math.min(Constants.SWERVE_SPEED_MULTIPLIER, 1), states[1].angle.getRadians());
                backLeftModule.set((feedForward.calculate(states[2].speedMetersPerSecond) + velocityPID[2].calculate(backLeftModule.getDriveVelocity(), states[2].speedMetersPerSecond)) * Math.min(Constants.SWERVE_SPEED_MULTIPLIER, 1), states[2].angle.getRadians()); 
                backRightModule.set((feedForward.calculate(states[3].speedMetersPerSecond) + velocityPID[3].calculate(backRightModule.getDriveVelocity(), states[3].speedMetersPerSecond)) * Math.min(Constants.SWERVE_SPEED_MULTIPLIER, 1), states[3].angle.getRadians()); 
        }
}