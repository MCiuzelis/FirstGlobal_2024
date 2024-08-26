package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.setIntakeState;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.setLiftState_UP;
import org.firstinspires.ftc.teamcode.commands.swollowBall;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
@Photon
@TeleOp(name = "servoTuner")

public class servoTuning extends TeleOpBase {

    @Override
    public void Init() {
        //driver gamepad commands:
        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD)));
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.RELEASE)));
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.BLOCK_INTAKE)));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        CommandScheduler.getInstance().run();

        if (gamepad1.left_bumper){
            robot.liftMotor_Left.setPower(-0.5);
            robot.liftMotor_Right.setPower(-0.5);
        }
        else if (gamepad1.right_bumper){
            robot.liftMotor_Left.setPower(1);
            robot.liftMotor_Right.setPower(1);
        }
        else{
            robot.liftMotor_Left.setPower(0);
            robot.liftMotor_Right.setPower(0);
        }
    }
}