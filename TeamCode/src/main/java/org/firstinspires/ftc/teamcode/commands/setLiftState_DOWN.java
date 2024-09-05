package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setLiftState_DOWN extends SequentialCommandGroup {
    public setLiftState_DOWN(LiftSubsystem lift){
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //if lift already up
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                new WaitCommand(1800),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD)
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
        addRequirements(lift);
    }
}