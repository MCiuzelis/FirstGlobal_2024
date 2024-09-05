package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setTopServoState extends InstantCommand {
    public setTopServoState(LiftSubsystem liftSubsystem, LiftSubsystem.TOP_SERVO_POSITION position){
        super(()-> liftSubsystem.update(position));
    }
}
