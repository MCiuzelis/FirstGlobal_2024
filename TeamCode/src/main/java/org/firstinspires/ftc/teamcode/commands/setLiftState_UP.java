package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setLiftState_UP extends SequentialCommandGroup {
    public setLiftState_UP(LiftSubsystem lift){
        addCommands(
                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.UP)
        );
        addRequirements(lift);
    }
}
