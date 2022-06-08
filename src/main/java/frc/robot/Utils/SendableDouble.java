package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * @author Stephen Oz
 */
public class SendableDouble implements Sendable {
    String key = "";
    
    DoubleSupplier getter;
    DoubleConsumer setter;
    double value;
    public SendableDouble(String key) {
        this.key = key;
    }
    
    public SendableDouble(DoubleSupplier supplier, DoubleConsumer consumer, Double value, String key) {
        this.getter = supplier;
        this.setter = consumer;
        this.value = value;
        this.key = key;
    }

    public double get() {
        return this.getter.getAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(key, getter, setter);
        builder.setSmartDashboardType("Double");
    }
    
}
