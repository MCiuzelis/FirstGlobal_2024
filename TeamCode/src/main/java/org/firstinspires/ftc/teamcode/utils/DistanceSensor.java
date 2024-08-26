package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class DistanceSensor {
    private final com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;
    private final AtomicReference<Double> distanceValue = new AtomicReference<>(0.0);
    private final AtomicBoolean inFlight = new AtomicBoolean(false);

    public DistanceSensor(HardwareMap hardwareMap, String sensorName){
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, sensorName);
    }

    private CompletableFuture<Double> getRawDistanceAsync(){
        CompletableFuture<Double> completableFuture = new CompletableFuture<>();

        Executors.newCachedThreadPool().submit(() -> {
            Thread.sleep(4);
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            completableFuture.complete(distance);
            return distance;
        });

        return completableFuture;
    }

    public void process() {
        if (!inFlight.get()) {
            inFlight.set(true);
            getRawDistanceAsync().thenAccept(distance ->{
                distanceValue.set(distance);
                inFlight.set(false);
            });
        }
    }

    public double getDistance() {
        return distanceValue.get();
    }
}