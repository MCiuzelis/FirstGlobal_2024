package org.firstinspires.ftc.teamcode.opMode;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Photon
@TeleOp(name = "tankTest")

public class TestOpMode extends TeleOpBase {

    @Override
    public void Init() {
    }

    @Override
    public void Start() {
        robot.startIMUThread(this);
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        tank.loop(driver.getGamepadInput());
    }
}