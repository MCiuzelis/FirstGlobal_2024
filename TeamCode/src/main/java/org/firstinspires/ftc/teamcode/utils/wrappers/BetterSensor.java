package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.DistanceProvider;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class BetterSensor {
    private final DistanceProvider distanceProvider;
    private final AtomicReference<Double> value = new AtomicReference<>(0.0);
    private final AtomicBoolean inFlight = new AtomicBoolean(false);
    private final LowPassFilter filter = new LowPassFilter(0.8);

    public BetterSensor(DistanceProvider distanceProvider) {
        this.distanceProvider = distanceProvider;
    }

    private CompletableFuture<Double> getRawDistanceAsync(){
        CompletableFuture<Double> completableFuture = new CompletableFuture<>();

        Executors.newCachedThreadPool().submit(() -> {
            Thread.sleep(3);
            double distance = filter.estimate(distanceProvider.getDistance(DistanceUnit.CM));
            completableFuture.complete(distance);
            return distance;
        });

        return completableFuture;
    }

    public void process() {
        if (!inFlight.get()) {
            inFlight.set(true);
            getRawDistanceAsync().thenAccept(distance -> {
                value.set(distance);
                inFlight.set(false);
            });
        }
    }

    public double getData() {
        return value.get();
    }
}
