package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class kickBall extends SequentialCommandGroup {
    public kickBall(IntakeSubsystem intake){
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //IF HOLDING BALL
                                new setIntakeSpeedCommand(intake, -1),
                                new WaitCommand(300),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),
                                new WaitCommand(800),
                                new setIntakeSpeedCommand(intake, 0),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
                        ),
                        new SequentialCommandGroup(
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                                new setIntakeSpeedCommand(intake, IntakeSubsystem.nominalSpeed),

                                new WaitCommand(300),

                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                new setIntakeSpeedCommand(intake, 0)
                        ),
                        ()-> intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL)
                )
        );
        addRequirements(intake);
    }
}
