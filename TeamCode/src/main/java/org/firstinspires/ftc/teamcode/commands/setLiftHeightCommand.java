package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setLiftHeightCommand extends InstantCommand {
    public setLiftHeightCommand (LiftSubsystem liftSubsystem, LiftSubsystem.TARGET_POSITION position){
        super(()-> liftSubsystem.setTargetPosition(position));
    }
}