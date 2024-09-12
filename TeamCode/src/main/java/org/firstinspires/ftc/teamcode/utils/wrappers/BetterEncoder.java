package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class BetterEncoder extends CuttleEncoder {
    private double offset = 0;
    private double position = 0;

    public BetterEncoder(CuttleRevHub revHub, int port, double countsPerRevolution) {
        super(revHub, port, countsPerRevolution);
    }

    public void reset(){
        offset = super.getRotation();
    }


    public void updatePosition(){
        position = super.getRotation() - offset;
    }

    public double getVelocity(){
        return super.getVelocity();
    }

    public double getPosition(){
        return position;
    }
}
