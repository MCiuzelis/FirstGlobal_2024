package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.depositBall;
import org.firstinspires.ftc.teamcode.commands.jerkCommand;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.setLiftState_LOW_MID_HIGH;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Inside;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Outside;
import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.overrideLiftPower;
import org.firstinspires.ftc.teamcode.commands.wrappers.resetLiftEncoder;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setTopServoState;
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

        //MAIN DRIVER COMMANDS:
        driver.getGamepadButton(driver.square)
                .toggleWhenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE)),
                                   ()-> schedule(new setLiftState_DOWN(intake, lift, driver))

                );
        driver.getGamepadButton(driver.cross)
                .whenPressed(()-> schedule(new setLiftState_LOW_MID_HIGH(intake, lift, setLiftState_LOW_MID_HIGH.STATE.LOW)
                ));
        driver.getGamepadButton(driver.circle)
                .whenPressed(()-> schedule(new setLiftState_LOW_MID_HIGH(intake, lift, setLiftState_LOW_MID_HIGH.STATE.MID)

                ));
        driver.getGamepadButton(driver.triangle)
                .whenPressed(()-> schedule(new setLiftState_LOW_MID_HIGH(intake, lift, setLiftState_LOW_MID_HIGH.STATE.HIGH)
                ));
        driver.getGamepadButton(driver.dpadDown)
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new rumbleCommand(driver, 100),
                        new intakeBall(lift, intake, driver),
                        ()->lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)
                )));
        driver.getGamepadButton(driver.dpadLeft)
                .whenPressed(()-> schedule(new swallowBall_Outside(lift, intake)

                ));
        driver.getGamepadButton(driver.dpadRight)
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new swallowBall_Inside(lift, intake),
                        new emptyCommand(),
                        ()-> !(intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE) ||
                               intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE
                )))));
        driver.getGamepadButton(driver.dpadUp)
                .whenPressed(()-> schedule(new depositBall(intake, lift)
                ));
        driver.getGamepadButton(driver.M1)
                .whenPressed(()-> schedule(new SequentialCommandGroup(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                        new setIntakeSpeedCommand(intake, 0),
                        new WaitCommand(700),
                        new InstantCommand(()-> intake.calibrateAngle())
                )));
        driver.getGamepadButton(driver.M2)
                .toggleWhenPressed(()-> schedule(new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.HOLD)),
                                   ()-> schedule(new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.FOLDED))
                );
        driver.getGamepadButton(driver.leftBumper)
                        .whenPressed(()-> schedule(new kickBall(intake)
        ));


        jerkCommand jerkCommandInstance = new jerkCommand(lift);
        driver.getGamepadButton(driver.rightBumper)
                .whileHeld(() -> CommandScheduler.getInstance().schedule(jerkCommandInstance))
                .whenReleased(() -> {
                    CommandScheduler.getInstance().cancel(jerkCommandInstance);
                    schedule(new overrideLiftPower(lift, 0));
        });




        //SECOND DRIVER OVERRIDES
        assistant.getGamepadButton(assistant.triangle)
                .whenPressed(()-> schedule(new resetLiftEncoder(lift, LiftSubsystem.highPosition))
                );
        assistant.getGamepadButton(assistant.circle)
                .whenPressed(()-> schedule(new resetLiftEncoder(lift, LiftSubsystem.lowPosition))
                );
        assistant.getGamepadButton(assistant.cross)
                .whenPressed(()-> schedule(new resetLiftEncoder(lift, LiftSubsystem.initialPosition))
                );
        assistant.getGamepadButton(assistant.dpadUp)
                .whileHeld(()-> schedule(new overrideLiftPower(lift, 0.6)))
                .whenReleased(()-> schedule(new overrideLiftPower(lift, 0))
                );
        assistant.getGamepadButton(assistant.dpadDown)
                .whileHeld(()-> schedule(new overrideLiftPower(lift, -0.4)))
                .whenReleased(()-> schedule(new overrideLiftPower(lift, 0))
                );
        assistant.getGamepadButton(assistant.dpadLeft)
                .whenPressed(()-> schedule(new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP))
                );
        assistant.getGamepadButton(assistant.dpadRight)
                .whenPressed(()-> schedule(new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN))
                );
        assistant.getGamepadButton(assistant.leftBumper)
                .whileHeld(()-> schedule(new setIntakeSpeedCommand(intake, 0.5)))
                .whenReleased(()-> schedule(new setIntakeSpeedCommand(intake, 0))
                );
        assistant.getGamepadButton(assistant.rightBumper)
                .whileHeld(()-> schedule(new setIntakeSpeedCommand(intake, -0.5)))
                .whenReleased(()-> schedule(new setIntakeSpeedCommand(intake, 0))
                );
        assistant.getGamepadButton(assistant.M1)
                .whenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE))
                );
        assistant.getGamepadButton(assistant.M2)
                .whenPressed(()-> schedule(new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD))
                );
    }

    @Override
    public void Start() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),
                new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.FOLDED)
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

        telemetry.addData("dsitance to ball", robot.colorSensor.getDistance(DistanceUnit.CM));
    }
}