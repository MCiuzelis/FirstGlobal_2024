package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class overrideLiftPower extends InstantCommand {
    public overrideLiftPower(LiftSubsystem liftSubsystem, double power){
        super(()-> liftSubsystem.overrideLiftPower(power));
    }
}
