package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class setIntakeSpeedCommand extends InstantCommand {
    public setIntakeSpeedCommand(IntakeSubsystem intake, double speed){
        super(()-> intake.setSpeed(speed));
    }
}
