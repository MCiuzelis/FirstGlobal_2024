package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Photon
@TeleOp(name = "tankTest")

public class TestOpMode extends TeleOpBase {
    public static double power = 0.5;

    @Override
    public void Init() {
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        tank.loop(driver.getGamepadInput());

        if (gamepad1.triangle){
            robot.liftMotor_Left.setPower(power);
            robot.liftMotor_Right.setPower(power);
        }
        else if (gamepad1.cross){
            robot.liftMotor_Left.setPower(-power / 2);
            robot.liftMotor_Right.setPower(-power / 2);
        }
        else{
            robot.liftMotor_Left.setPower(0);
            robot.liftMotor_Right.setPower(0);
        }
    }
}