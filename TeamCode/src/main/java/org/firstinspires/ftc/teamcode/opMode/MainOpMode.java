package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.swollowBall;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.setLiftState_UP;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.setIntakeState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
@Photon
@TeleOp(name = "mainTeleOP")

public class MainOpMode extends TeleOpBase {

    @Override
    public void Init() {
        CommandScheduler.getInstance().schedule(
                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD)
        );
        CommandScheduler.getInstance().run(); //runs scheduled commands

        //driver gamepad commands:
        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new setLiftState_UP(lift)));
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new setLiftState_DOWN(lift)));
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new swollowBall(lift, intake)));
        driver.getGamepadButton(driver.circle)
                .toggleWhenPressed(()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.UP)),
                                   ()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.DOWN)));
        driver.getGamepadButton(driver.leftBumper)
                .whenPressed(()-> schedule(new intakeBall(lift, intake, driver)));
        driver.getGamepadButton(driver.rightBumper)
                .whenPressed(()-> schedule(new kickBall(intake)));
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new InstantCommand(()-> lift.offsetTarget(10))));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new InstantCommand(()-> lift.offsetTarget(-10))));
    }

    @Override
    public void Start() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN)
        ));
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        intake.setAngleOffset(gamepad1.left_trigger);

        CommandScheduler.getInstance().run();
        tank.loop(driver.getGamepadInput(lift.getHeight()));

        robot.distanceSensor.process();

        telemetry.addData("lift", robot.encoder_liftPosition.getRotation());
        telemetry.addData("current", robot.controlHub.getBatteryCurrent());
        telemetry.addData("distance", robot.distanceSensor.getDistance());



        /*if (gamepad1.left_bumper){
            robot.liftMotor_Left.setPower(-power/2);
            robot.liftMotor_Right.setPower(-power/2);
        }
        else if (gamepad1.right_bumper){
            robot.liftMotor_Left.setPower(power);
            robot.liftMotor_Right.setPower(power);
        }
        else if(gamepad1.dpad_right){
            robot.liftMotor_Left.setPower(0);
            robot.liftMotor_Right.setPower(0);
        }*/
    }
}