package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterGamepad;

public class setLiftState_DOWN extends SequentialCommandGroup {
    public setLiftState_DOWN(IntakeSubsystem intake, LiftSubsystem lift, BetterGamepad driver){
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //if lift already up
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                new WaitCommand(1200),
                                new rumbleCommand(driver, 100),
                                new WaitCommand(400),

                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW),
                                new WaitCommand(50),

                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),

                                new ConditionalCommand(
                                        new transferBall(intake, lift),
                                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),
                                        ()-> lift.getBallState().equals(LiftSubsystem.BALL_STATE.IN_TRANSFER))
                        ),
                        new SequentialCommandGroup(
                                //lift down or in the middle of travel
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),

                                new ConditionalCommand(
                                        //WAIT FOR RETRACTION IF IN TRANSFER POS
                                        new SequentialCommandGroup(
                                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                                                new WaitCommand(500)
                                        ),
                                        new emptyCommand(),
                                        lift::areServosExtended
                                )
                        ),
                        lift::liftReachedPosition
                )
        );
        addRequirements(intake, lift);
    }
}