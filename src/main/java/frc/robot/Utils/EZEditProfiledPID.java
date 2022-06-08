package frc.robot.Utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EZEditProfiledPID extends ProfiledPIDController {
    String name;
    private EZEditProfiledPID(double kp, double ki, double kd, Constraints constraints) {
        super(kp, ki, kd, constraints);
    }

    public EZEditProfiledPID(double kp, double ki, double kd, Constraints constraints, String name) {
        super(kp, ki, kd, constraints);
        SendableRegistry.addLW(this, name);
        this.name = name;
    }

    @Override
    public void setP(double kp) {
        SmartDashboard.putData(this);
        super.setP(kp);
    }

    @Override
    public void setI(double ki) {
        SmartDashboard.putData(this);
        super.setI(ki);
    }

    @Override
    public void setD(double kd) {
        SmartDashboard.putData(this);
        super.setD(kd);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("P", super::getP, super::setP);
        builder.addDoubleProperty("I", super::getI, super::setI);
        builder.addDoubleProperty("D", super::getD, super::setD);
        builder.setSmartDashboardType("EZEditProfiledPID");
    }
}
