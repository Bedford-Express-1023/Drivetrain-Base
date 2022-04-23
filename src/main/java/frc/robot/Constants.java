package frc.robot;

import frc.robot.Utils.Gains;
import frc.robot.Utils.LinearInterpolator;

public class Constants {
    //////////////////////
    //DRIVETRAIN CONSTANTS
    //////////////////////
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.521;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.521;

    public static final double SWERVE_MAX_VOLTAGE = 12;
    public static final double SWERVE_SPEED_MULTIPLIER = 1; //use to limit speed
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.25; //Literally a measure of how fast the swerves could possibly go
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 4.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 4.0);

    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 42;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(264);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(289);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(4.9);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(342);

    ///////////////////
    //SHOOTER CONSTANTS
    ///////////////////
    public static final double kNominalVoltage = 12.0;
    public static final int kFalconCPR = 2048; //pulses per rotation
    public static final int kCANTimeoutMs = 250;

    public final static int shooterPID = 0;
    public static int shooterTimeout = 100;
    public static final Gains kBottomGains = new Gains(0.06 , 0.0, 0.0, 0.05, 0, 0.8);
    public static final Gains kTopGains = new Gains(0.05, 0.0, 0.0, 0.05, 0, 0.8);
    public final static int SLOT_0 = 0;
    public final static int ShooterLowRPM = 10000;
    public final static int ShooterHighRPM = 10000;

    public static final class TargetConstants{
		public static final double[][] HOOD_ARRAY = {
                {-18.1, 190},
                {-17.4, 190},
                {-16.1, 195},
                {-15.6, 175},
                {-15.1, 165},
                {-14.9, 170},
                {-14, 165},
                {-12.5,180},
                {-11.1, 175},
                {-10.1, 175},
                {-9.5, 165},
                {-8.5,155},
                {-8,149},
                {-7.2, 140},
				{-5.8,130},
                {-4.2,110},
                {-2.4, 100},
				{-1.1,110},
				{0,110},
				{4.5, 97},
				{5,90},
				{7,80},
                {7.5,80},
				{10.7,75},
                {12.2,80},
                {15.5,65},
                {17.6,65}
			};

		public static final double[][] SHOOTER_TOP_ARRAY = {
            {17.6,7500},
            {15.5, 7900},
            {12.2,8100},
			{10.7,8100},
            {7.5,8400},
            {5.5,8650},
			{4,8700},
			{0.0,8600},
			{-1.1,8600},
            {-2.4, 8600},
            {-4.2,8600},
			{-5.8,8600},
            {-7.2, 9100},
            {-8,9361},
            {-8.5, 9900},
            {-9.5, 10100},
            {-10.1, 10300},
            {-11.1, 10300},
            {-12.5,10600},
            {-14, 10658},
            {-14.9,10958},
            {-15.1, 11058},
            {-15.6, 11200},
            {-16.1, 11300},
            {-17.4, 12000},
            {-18.1, 12500}
            
		};

        public static final double[][] SHOOTER_BOTTOM_ARRAY = {
            {17.6,11500},
            {15.5,11500},
            {12.2,11100},
			{10.7,11100},
            {7.5,11600},
            {5.5,11450},
			{4,11300},
			{0.0,11800},
			{-1.1,12500},
            {-2.4, 12500},
            {-4.2, 12600},
			{-5.8,12200},
            {-7.2, 11900},
            {-8, 12880},
            {-8.5, 12300},
            {-9.5, 12750},
            {-10.1, 13300},
            {-11.1,12202},
            {-12.5, 13800},
            {-14,14200 },
            {-14.9, 13785},
            {-15.1, 14102},
            {-15.6, 14402},
            {-16.1, 14900},
            {-17.4, 14902},
            {-18.1, 15100}
        };

		public static final LinearInterpolator HOOD_INTERPOLATOR = new LinearInterpolator(HOOD_ARRAY);
		public static final LinearInterpolator SHOOTER_TOP_SPEED_INTERPOLATOR = new LinearInterpolator(SHOOTER_TOP_ARRAY);
        public static final LinearInterpolator SHOOTER_BOTTOM_SPEED_INTERPOLATOR = new LinearInterpolator(SHOOTER_BOTTOM_ARRAY);
    }
}