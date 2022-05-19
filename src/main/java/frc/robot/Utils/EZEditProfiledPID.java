package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EZEditProfiledPID extends ProfiledPIDController {


    public EZEditProfiledPID(double kp, double ki, double kd, Constraints constraints, ShuffleboardTab displayTab, String name) {
        super(kp, ki, kd, constraints);
        ShuffleboardLayout layout = displayTab.getLayout(name, BuiltInLayouts.kList);
        layout.add(new SendableDouble(this::getP, this::setP, kp));
        layout.add(new SendableDouble(this::getI, this::setI, ki));
        layout.add(new SendableDouble(this::getD, this::setD, kd));
    }

    String key;
    DoubleSupplier getter;
    DoubleConsumer setter;
    double value;
    
}
