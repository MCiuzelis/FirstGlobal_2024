package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class setIntakeVelocityCommand extends InstantCommand {
    public setIntakeVelocityCommand(IntakeSubsystem intake, double velocity){
        super(()-> intake.setIntakeTargetVelocity(velocity));
    }
}
