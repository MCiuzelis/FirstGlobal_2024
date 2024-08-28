package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Rev2mDistanceSensorProvider implements DistanceProvider {
    private final Rev2mDistanceSensor sensor;

    public Rev2mDistanceSensorProvider(HardwareMap hardwareMap, String sensorName) {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, sensorName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }
}
