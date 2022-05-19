package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EZEditPID extends PIDController {
    
    private EZEditPID(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public EZEditPID(double kp, double ki, double kd, ShuffleboardTab displayTab, String name) {
        super(kp, ki, kd);
        ShuffleboardLayout layout = displayTab.getLayout(name, BuiltInLayouts.kList);
        layout.add(new SendableDouble(super::getP, super::setP, kp));
        layout.add(new SendableDouble(super::getI, super::setI, ki));
        layout.add(new SendableDouble(super::getD, super::setD, kd));
    }

    String key;
    DoubleSupplier getter;
    DoubleConsumer setter;
    double value;
    
}
