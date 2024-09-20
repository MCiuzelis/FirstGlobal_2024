package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.overrideLiftPower;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class jerkCommand extends SequentialCommandGroup {
    public jerkCommand(LiftSubsystem lift){
        addCommands(
                new RepeatCommand(
                        new SequentialCommandGroup(
                            new overrideLiftPower(lift, -0.5),
                            new WaitCommand(90),
                            new overrideLiftPower(lift, 0.65),
                            new WaitCommand(100)
                        )
                ));
        addRequirements(lift);
    }
}
