package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class BetterMotor extends CuttleMotor {
    private double prevPower = 0;

    public BetterMotor(CuttleRevHub revHub, int port) {
        super(revHub, port);
    }

    @Override
    public void setPower(double power) {
        if (power != prevPower){
            prevPower = power;
            super.setPower(power);
        }
    }
}
