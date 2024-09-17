package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class kickBall extends SequentialCommandGroup {
    public kickBall(IntakeSubsystem intake, LiftSubsystem lift){
        addCommands(
                        new SequentialCommandGroup(
                                //IF HOLDING BALL
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                //IF BALL INSIDE
                                                new setIntakeSpeedCommand(intake, -1),
                                                new WaitCommand(300),
                                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),
                                                new WaitCommand(800),
                                                new setIntakeSpeedCommand(intake, 0),
                                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD)
                                        ),
                                        new SequentialCommandGroup(
                                                //IF BALL OUTSIDE
                                                new setIntakeSpeedCommand(intake, 1),
                                                new WaitCommand(600),
                                                new setIntakeSpeedCommand(intake, 0)
                                        ),
                                        ()-> intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE)
                                )
                        )
        );
        addRequirements(intake, lift);
    }
}