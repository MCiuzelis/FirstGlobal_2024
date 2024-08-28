package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class BetterServo extends CuttleServo {
    private Direction direction = Direction.FORWARD;

    public BetterServo(CuttleRevHub revHub, int servoPort) {
        super(revHub, servoPort);
    }

    public BetterServo(CuttleRevHub revHub, int servoPort, Direction direction) {
        super(revHub, servoPort);
        this.direction = direction;
    }

    @Override
    public void setPosition(double position){
        if (direction.equals(Direction.REVERSE)){
            super.setPosition(1 - position);
        }
        else{
            super.setPosition(position);
        }
    }

    public void setDirection(Direction dir){
        direction = dir;
    }

    public enum Direction{
        FORWARD,
        REVERSE
    }
}
