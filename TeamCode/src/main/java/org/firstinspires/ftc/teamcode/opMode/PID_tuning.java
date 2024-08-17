package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.setIntakeState;
import org.firstinspires.ftc.teamcode.commands.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Disabled
@Config
@Photon
@TeleOp(name = "PID")

public class PID_tuning extends TeleOpBase {
    public static double power = 0.5;

    @Override
    public void Init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.UP)));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.DOWN)));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        intake.setAngleOffset(gamepad1.left_trigger);
        CommandScheduler.getInstance().run();
    }
}