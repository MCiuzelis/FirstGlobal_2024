package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Inside;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Outside;
import org.firstinspires.ftc.teamcode.commands.transferBall;
import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
@Photon
@TeleOp(name = "MainTeleOP")

public class MainOpMode extends TeleOpBase {

    @Override
    public void Init() {
        lift.update(LiftSubsystem.BUCKET_SERVO_POSITION.HOLD);
        intake.calibrateAngle();

        //driver gamepad commands:
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new setLiftState_DOWN(lift)
                ));
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW)
                )));
        driver.getGamepadButton(driver.circle)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.MID)
                )));
        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.HIGH)
                )));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new rumbleCommand(driver, 100),
                        new intakeBall(lift, intake, driver),
                        ()->lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)
                )));
        driver.getGamepadButton(driver.dpadLeft)
                .whenPressed(()-> schedule(
                        new swallowBall_Outside(lift, intake)
                ));
        driver.getGamepadButton(driver.dpadRight)
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new swallowBall_Inside(lift, intake),
                        new emptyCommand(),
                        ()-> !(intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE) ||
                               intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE
                )))));
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new kickBall(intake, lift)
                ));
        driver.getGamepadButton(driver.M1)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                        new setIntakeSpeedCommand(intake, 0),
                        new WaitCommand(700),
                        new InstantCommand(()-> intake.calibrateAngle())
                )));
        driver.getGamepadButton(driver.M2)
                .whenPressed(()-> schedule(new transferBall(lift, intake)
                ));
    }

    @Override
    public void Start() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN)
        ));
    }

    @Override
    public void Loop() {
        robot.controlHub.pullBulkData();
        robot.encoder_liftPosition.updatePosition();
        robot.encoder_intake_Angle.updatePosition();
        intake.setAngleOffset(driver.leftTrigger());

        CommandScheduler.getInstance().run();

        double driveMultiplier = 1;
        if (driver.rightTrigger() < 0.5){
            driveMultiplier = lift.mapValue(lift.getHeight(), 6 / 82.5, 1, 1, 0.3);
        }

        tank.loop(driver.getGamepadInput(driveMultiplier, driveMultiplier));
        //robot.distanceSensor.process();
        //robot.spinyCurrentSensor.process();
        //telemetry.addData("Robot's current draw: ", robot.controlHub.getBatteryCurrent());
    }
}