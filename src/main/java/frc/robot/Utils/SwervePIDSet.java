package frc.robot.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * @author Stephen Oz
 */
public class SwervePIDSet implements Sendable {
    public String name = "SwerveModulePIDs";
    private PIDController[] pidControllers = new PIDController[4];
    public SwervePIDSet(double kp, double ki, double kd) {
        pidControllers[0] = new PIDController(kp, ki, kd);
        pidControllers[1] = new PIDController(kp, ki, kd);
        pidControllers[2] = new PIDController(kp, ki, kd);
        pidControllers[3] = new PIDController(kp, ki, kd);
        SmartDashboard.putData(this);
    }

    public void setP(double kP) {
        pidControllers[0].setP(kP);
        pidControllers[1].setP(kP);
        pidControllers[2].setP(kP);
        pidControllers[3].setP(kP);
    }

    public double getP() {
        return pidControllers[0].getP();
    }

    public void setI(double kI) {
        pidControllers[0].setI(kI);
        pidControllers[1].setI(kI);
        pidControllers[2].setI(kI);
        pidControllers[3].setI(kI);
    }

    public double getI() {
        return pidControllers[0].getI();
    }

    public void setD(double kD) {
        pidControllers[0].setD(kD);
        pidControllers[1].setD(kD);
        pidControllers[2].setD(kD);
        pidControllers[3].setD(kD);
    }

    public double getD() {
        return pidControllers[0].getD();
    }

    /**
     * 
     * @param module which module to calculate for, 0:FL, 1:FR, 2:BL, 3:BR
     * @return
     */
    public double calculate(int module, double measurement, double setpoint) {
        return pidControllers[module].calculate(measurement);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwervePIDSet");
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
    }
}
