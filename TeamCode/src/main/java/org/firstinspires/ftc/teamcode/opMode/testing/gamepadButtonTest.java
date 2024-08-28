package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opMode.TeleOpBase;

@Config
@Photon
@TeleOp(name = "gamapedTester")

public class gamepadButtonTest extends TeleOpBase {

    @Override
    public void Init() {
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        telemetry.addData("options: ", gamepad1.options);
        telemetry.addData("share: ", gamepad1.share);

    }
}