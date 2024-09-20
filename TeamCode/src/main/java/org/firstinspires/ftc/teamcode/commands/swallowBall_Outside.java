package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class swallowBall_Outside extends SequentialCommandGroup {
    public swallowBall_Outside(LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                new WaitCommand(100),
                new setIntakeSpeedCommand(intake, -0.7),
                new WaitUntilCommand(()-> intake.overCurrentTriggered(0.6)),
                new setIntakeSpeedCommand(intake, -0.4),

                new WaitCommand(200),
                new setIntakeSpeedCommand(intake, -0.085),
                new WaitCommand(100),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE),
                new InstantCommand(()-> lift.setBallState(LiftSubsystem.BALL_STATE.IN_TRANSFER))
        );
        addRequirements(lift, intake);
    }
}
