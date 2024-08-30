package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterMotor;

public class CoreHexMotorCurrentProvider implements DistanceProvider {
    private final BetterMotor motor;

    public CoreHexMotorCurrentProvider(BetterMotor motor) {
        this.motor = motor;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return motor.getCurrent();
    }
}
