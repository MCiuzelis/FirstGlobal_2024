package org.firstinspires.ftc.teamcode.opMode.testing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.swallowBall_Outside;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.opMode.TeleOpBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

//@Disabled
@Config
@Photon
@TeleOp(name = "INTAKE test")

public class Intake_tuning extends TeleOpBase {
    public static double power = 0.5;

    @Override
    public void Init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                        new setIntakeSpeedCommand(intake, 0)
                )));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                        new setIntakeSpeedCommand(intake, 0)
                )));
        driver.getGamepadButton(driver.dpadLeft)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.TRANSFER),
                        new setIntakeSpeedCommand(intake, 0)
                )));
        driver.getGamepadButton(driver.dpadLeft)
                .whenPressed(()-> schedule(
                        new swallowBall_Outside(lift, intake)
                ));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        intake.setAngleOffset(gamepad1.left_trigger);
        robot.encoder_liftPosition.updatePosition();
        robot.encoder_intake_Angle.updatePosition();
        CommandScheduler.getInstance().run();
    }
}