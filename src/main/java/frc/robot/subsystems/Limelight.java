package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/**
 * Helper class for getting limelight values, calculating distance, etc.
 */
public abstract class Limelight {
    public double DistanceToTarget() {
        return 
            Constants.TARGET_HEIGHT_ABOVE_LIMELIGHT / 
            Constants.LIMELIGHT_MOUNTING_ANGLE.minus(
                Rotation2d.fromDegrees(ty())).getTan();
    }

    /**
     * @return the limelight's ty value, its vertical angle to the target 
     */
    public static double ty() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); 
    }

    /**
     * @return the limelight's tx value, its horizontal angle to the target 
     */
    public static double tx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
    }

    /**
     * @return the limelight's tv value, whether is has a valid target or not
     */    
    public static boolean tv() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 0)
        {
            return false;
        }
        return true;
    }
}