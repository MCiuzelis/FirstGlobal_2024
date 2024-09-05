package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class swallowBall_Outside extends SequentialCommandGroup {
    public swallowBall_Outside(LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                new WaitCommand(100),
                new setIntakeSpeedCommand(intake, -0.6),
                new WaitUntilCommand(()-> intake.overCurrentTriggered(-0.6)),
                new WaitCommand(60),
                new setIntakeSpeedCommand(intake, -0.15),
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE)
        );
        addRequirements(lift, intake);
    }
}
