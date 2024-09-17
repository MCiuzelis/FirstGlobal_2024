package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class resetLiftEncoder extends InstantCommand {
    public resetLiftEncoder(LiftSubsystem liftSubsystem, double currentPosition){
        super(()-> liftSubsystem.resetLiftEncoder(currentPosition));
    }
}
