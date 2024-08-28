package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevColorSensorV3Provider implements DistanceProvider {
    private final RevColorSensorV3 sensor;

    public RevColorSensorV3Provider(HardwareMap hardwareMap, String sensorName) {
        sensor = hardwareMap.get(RevColorSensorV3.class, sensorName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);  // Assuming this method exists for RevColorSensorV3
    }
}
