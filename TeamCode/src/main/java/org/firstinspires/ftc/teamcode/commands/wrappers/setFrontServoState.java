package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setFrontServoState extends InstantCommand {
    public setFrontServoState(LiftSubsystem liftSubsystem, LiftSubsystem.BUCKET_SERVO_POSITION position){
        super(()-> liftSubsystem.update(position));
    }
}
