package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.intakeBall;
import org.firstinspires.ftc.teamcode.commands.kickBall;
import org.firstinspires.ftc.teamcode.commands.setLiftState_DOWN;
import org.firstinspires.ftc.teamcode.commands.setLiftState_LOW_MID_HIGH;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Inside;
import org.firstinspires.ftc.teamcode.commands.swallowBall_Outside;
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
    //public static buttonSimulator currentButton = buttonSimulator.NONE;

    @Override
    public void Init() {
        lift.update(LiftSubsystem.BUCKET_SERVO_POSITION.HOLD);
        intake.calibrateAngle();

        //driver gamepad commands:
        driver.getGamepadButton(driver.square)
                .whenPressed(()-> schedule(new setLiftState_DOWN(intake, lift, driver)
                ));
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
                .whenPressed(()-> schedule(new ConditionalCommand(
                        new swallowBall_Outside(lift, intake),
                        new emptyCommand(),
                        ()-> !(intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE) ||
                                intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE
                )))));
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



//        if (currentButton.equals(buttonSimulator.DPAD_LEFT)){
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new swallowBall_Outside(lift, intake));
//        } else if (currentButton.equals(buttonSimulator.CROSS)){
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new setLiftState_LOW_MID_HIGH(intake, lift, setLiftState_LOW_MID_HIGH.STATE.LOW));
//        } else if (currentButton.equals(buttonSimulator.DPAD_RIGHT)) {
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new swallowBall_Inside(lift, intake));
//        } else if (currentButton.equals(buttonSimulator.DPAD_DOWN)) {
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new intakeBall(lift, intake, driver));
//        } else if (currentButton.equals(buttonSimulator.DPAD_UP)) {
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new kickBall(intake, lift));
//        } else if (currentButton.equals(buttonSimulator.SQUARE)) {
//            currentButton = buttonSimulator.NONE;
//            CommandScheduler.getInstance().schedule(new setLiftState_DOWN(intake, lift, driver));
//        }


        CommandScheduler.getInstance().run();

        double driveMultiplier = 1;
        if (driver.rightTrigger() < 0.5){
            driveMultiplier = lift.mapValue(lift.getHeight(), 6 / 82.5, 1, 1, 0.3);
        }

        tank.loop(driver.getGamepadInput(driveMultiplier, driveMultiplier));
    }

    private enum buttonSimulator{
        DPAD_LEFT,
        DPAD_RIGHT,
        DPAD_UP,
        DPAD_DOWN,
        CROSS,
        SQUARE,
        NONE
    }
}