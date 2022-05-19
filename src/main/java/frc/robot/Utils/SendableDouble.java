package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableDouble implements Sendable {
    String key = "";
    DoubleSupplier getter;
    DoubleConsumer setter;
    double value;
    public SendableDouble(String key) {
        this.key = key;
    }
    
    public SendableDouble(DoubleSupplier supplier, DoubleConsumer consumer, Double value) {
        this.getter = supplier;
        this.setter = consumer;
        this.value = value;
    }

    public void set(double value) { 
        this.value = value;
    }
    
    public double get() { 
        return value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(key, this::get, this::set);
    }
    
}
