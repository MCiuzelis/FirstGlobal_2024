package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class kickBall extends SequentialCommandGroup {
    public kickBall(IntakeSubsystem intake){
        addCommands(
                new setIntakeState(intake, setIntakeState.IntakeState.UP),
                new WaitCommand(300),
                new setIntakeState(intake, setIntakeState.IntakeState.DOWN)
        );
        addRequirements(intake);
    }
}
