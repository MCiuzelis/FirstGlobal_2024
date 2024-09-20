package org.firstinspires.ftc.teamcode.opMode.testing;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.angle_DOWN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.opMode.TeleOpBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.MotionProfile;

//@Disabled
@Config
@Photon
@TeleOp(name = "motionProfileTest")

public class intakeMotionProfileTest extends TeleOpBase {
    public static double targetPosition = 0;
    private double prevTargetPosition = 0;

    MotionProfile motionProfile;


    @Override
    public void Init() {
        motionProfile = new MotionProfile(telemetry);
        intake.calibrateAngle();
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        robot.encoder_intake_Angle.updatePosition();

        if (gamepad1.triangle){
            targetPosition = 200;
        } else if (gamepad1.cross) {
            targetPosition = 100;
        } else if (gamepad1.square) {
            targetPosition = 20;
        }

        if (targetPosition != prevTargetPosition){
            motionProfile.setTargetPosition(targetPosition, getAngleDegrees(robot.encoder_intake_Angle.getPosition()));
            prevTargetPosition = targetPosition;
        }

        robot.intake_AngleMotor.setPower(motionProfile.getPower(getAngleDegrees(robot.encoder_intake_Angle.getPosition())));

        //CommandScheduler.getInstance().run();
    }

    private double getAngleDegrees(double encoder){
        return Math.toDegrees(encoder) + angle_DOWN;
    }
}