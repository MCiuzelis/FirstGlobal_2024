package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class BetterServo extends CuttleServo {
    private Direction direction = Direction.FORWARD;
    private double prevPosition = 0;

    public BetterServo(CuttleRevHub revHub, int servoPort) {
        super(revHub, servoPort);
    }

    public BetterServo(CuttleRevHub revHub, int servoPort, Direction direction) {
        super(revHub, servoPort);
        this.direction = direction;
    }

    @Override
    public void setPosition(double position){
        if (prevPosition != position) {
            prevPosition = position;
            if (direction.equals(Direction.REVERSE)) {
                super.setPosition(1 - position);
            } else {
                super.setPosition(position);
            }
        }
    }

    public void setDirection(Direction dir){
        direction = dir;
    }

    @Override
    public double getPosition(){
        return prevPosition;
    }

    public enum Direction{
        FORWARD,
        REVERSE
    }
}
