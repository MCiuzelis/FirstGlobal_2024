package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class setIntakeAngleCommand extends InstantCommand {
    public setIntakeAngleCommand(IntakeSubsystem intake, IntakeSubsystem.INTAKE_ANGLE angle){
        super(()-> intake.setAngle(angle));
    }
}
