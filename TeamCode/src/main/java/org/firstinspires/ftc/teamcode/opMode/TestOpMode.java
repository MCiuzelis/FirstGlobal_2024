package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.setIntakeState;
import org.firstinspires.ftc.teamcode.commands.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
@Photon
@TeleOp(name = "tankTest")

public class TestOpMode extends TeleOpBase {
    public static double power = 0.5;

    @Override
    public void Init() {
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.UP)));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new setIntakeState(intake, setIntakeState.IntakeState.DOWN)));

//----------UNCOMMENT WHEN LIFT PID IS FIXED----------------------------------
//        driver.getGamepadButton(driver.triangle)
//                .whenPressed(()-> schedule(new setLiftHeightCommand(lift, LiftSubsystem.TARGET_POSITION.UP)));
//        driver.getGamepadButton(driver.cross)
//                .whenPressed(()-> schedule(new setLiftHeightCommand(lift, LiftSubsystem.TARGET_POSITION.DOWN)));
//        driver.getGamepadButton(driver.square)
//                .whenPressed(()-> schedule(new InstantCommand(()-> lift.offsetTarget(10))));
//        driver.getGamepadButton(driver.circle)
//                .whenPressed(()-> schedule(new InstantCommand(()-> lift.offsetTarget(-10))));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        intake.setAngleOffset(gamepad1.left_trigger);

        CommandScheduler.getInstance().run();
        tank.loop(driver.getGamepadInput());

        telemetry.addData("driveLeft", robot.encoder_driveBaseLeft.getRotation());
        telemetry.addData("driveRight", robot.encoder_driveBaseRight.getRotation());
        telemetry.addData("intake angle", robot.encoder_intake_Angle.getRotation());
        telemetry.addData("lift", robot.encoder_liftPosition.getRotation());


        if (gamepad1.left_bumper){
            robot.liftMotor_Left.setPower(power);
            robot.liftMotor_Right.setPower(power);
        }
        else if (gamepad1.right_bumper){
            robot.liftMotor_Left.setPower(-power / 2);
            robot.liftMotor_Right.setPower(-power / 2);
        }
        else{
            robot.liftMotor_Left.setPower(0);
            robot.liftMotor_Right.setPower(0);
        }
    }
}