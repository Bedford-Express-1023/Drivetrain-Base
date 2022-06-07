package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("P", super::getP, super::setP);
        builder.addDoubleProperty("I", super::getI, super::setI);
        builder.addDoubleProperty("D", super::getD, super::setD);
        builder.setSmartDashboardType("EZEditProfiledPID");
    }
}
