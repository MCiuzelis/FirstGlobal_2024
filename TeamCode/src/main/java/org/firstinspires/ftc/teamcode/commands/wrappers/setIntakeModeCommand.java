package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setIntakeModeCommand extends InstantCommand {
    public setIntakeModeCommand(IntakeSubsystem intake, IntakeSubsystem.MOTOR_MODE mode){
        super(()-> intake.setMode(mode));
    }
}
