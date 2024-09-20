package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class kickBall extends SequentialCommandGroup {
    public kickBall(IntakeSubsystem intake){
        addCommands(
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                new WaitCommand(400),
                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
        );
        addRequirements(intake);
    }
}
