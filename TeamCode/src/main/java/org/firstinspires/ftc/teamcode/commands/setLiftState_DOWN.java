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
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),

                                new ConditionalCommand(
                                        new transferBall(intake, lift, driver),
                                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),
                                        ()-> lift.getBallState().equals(LiftSubsystem.BALL_STATE.IN_TRANSFER)
                                )
                        ),
                        new SequentialCommandGroup(
                                //lift down or in the middle of travel
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),

                                new WaitCommand(150),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN)
                        ),
                        lift::liftReachedPosition
                )
        );
        addRequirements(intake, lift);
    }
}