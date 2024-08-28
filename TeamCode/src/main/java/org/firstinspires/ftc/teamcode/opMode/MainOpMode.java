package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.swollowBall;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
@Photon
@TeleOp(name = "mainTeleOP")

public class MainOpMode extends TeleOpBase {

    @Override
    public void Init() {
        lift.update(LiftSubsystem.SERVO_POSITION.HOLD);
        intake.calibrateAngle();

        //driver gamepad commands:
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new setLiftState_DOWN(lift)
                ));
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW)
                )));
        driver.getGamepadButton(driver.circle)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.MID)
                )));
        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.HIGH)
                )));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new rumbleCommand(driver, 100),
                        new intakeBall(lift, intake, driver),
                        ()->lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)
                )));
        driver.getGamepadButton(driver.dpadLeft)
                .whenPressed(()-> schedule(new swollowBall(lift, intake)
                ));
        driver.getGamepadButton(driver.dpadRight)
                .whenPressed(()-> schedule(new swollowBall(lift, intake)
                ));
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new kickBall(intake)
                ));
        driver.getGamepadButton(driver.rightStickButton)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                        new setIntakeSpeedCommand(intake, 0)
                )));
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

        double driveMultiplier = 1;
        if (!driver.getButton(driver.leftStickButton)){
            driveMultiplier = lift.mapValue(lift.getHeight());
        }

        tank.loop(driver.getGamepadInput(driveMultiplier, driveMultiplier));
        robot.sensor.process();

        telemetry.addData("Robot's current draw: ", robot.controlHub.getBatteryCurrent());
    }
}