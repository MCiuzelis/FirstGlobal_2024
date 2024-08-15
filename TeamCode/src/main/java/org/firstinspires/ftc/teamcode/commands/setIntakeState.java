package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class setIntakeState extends SequentialCommandGroup {
    public setIntakeState(IntakeSubsystem intake, IntakeState state) {
        addCommands(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new setIntakeSpeedCommand(intake, IntakeSubsystem.nominalSpeed),
                                new WaitCommand(200),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP)
                        ),
                        new ParallelCommandGroup(
                                new setIntakeSpeedCommand(intake, 0),
                                new WaitCommand(80),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
                        ),
                        () -> state.equals(IntakeState.UP)
                )
        );
        addRequirements(intake);
    }

    public enum IntakeState {
        UP,
        DOWN
    }
}

