package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class swallowBall_Outside extends SequentialCommandGroup {
    public swallowBall_Outside(LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                new WaitCommand(100),
//                new setIntakeModeCommand(intake, IntakeSubsystem.MOTOR_MODE.SET_VELOCITY),
//                new setIntakeSpeedCommand(intake, -0.6),
//                new WaitUntilCommand(()-> intake.overCurrentTriggered(0.7)),
//                new WaitCommand(80),
//                new setIntakeSpeedCommand(intake, 0),
//                new WaitCommand(300),
                new setIntakeModeCommand(intake, IntakeSubsystem.MOTOR_MODE.HOLD_POSITION),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE),
                new InstantCommand(()-> lift.setBallState(LiftSubsystem.BALL_STATE.IN_TRANSFER))
        );
        addRequirements(lift, intake);
    }
}
