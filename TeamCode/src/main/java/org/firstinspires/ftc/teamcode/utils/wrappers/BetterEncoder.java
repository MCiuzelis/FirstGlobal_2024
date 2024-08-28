package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class BetterEncoder extends CuttleEncoder {
    private double offset = 0;

    public BetterEncoder(CuttleRevHub revHub, int port, double countsPerRevolution) {
        super(revHub, port, countsPerRevolution);
    }

    public void reset(){
        offset = super.getRotation();
    }

    @Override
    public double getRotation(){
        return super.getRotation() - offset;
    }
}
