package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.opMode.TeleOpBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Disabled
@Config
@Photon
@TeleOp(name = "spin_Motor_feedforward_tuner")

public class spinyMotorVelocityTuning extends TeleOpBase {
    public static double velocity = 0;
    public static double f = 0;



    @Override
    public void Init() {
        intake.calibrateAngle();


        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP)));
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)));
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.FOLDED)));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        robot.encoder_intake_Angle.updatePosition();

        telemetry.addData("intakeSpinnyEncoder: ", robot.encoder_intake_Speed.getPosition());
        CommandScheduler.getInstance().run();
    }
}