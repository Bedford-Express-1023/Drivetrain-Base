package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EZEditPID extends PIDController {
    String name;
    private EZEditPID(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public EZEditPID(double kp, double ki, double kd, String name) {
        super(kp, ki, kd);
        SmartDashboard.putData(name, this);
        this.name = name;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("EZEditPID");
        builder.addDoubleProperty("P", super::getP, super::setP);
        builder.addDoubleProperty("I", super::getI, super::setI);
        builder.addDoubleProperty("D", super::getD, super::setD);
    }
}
