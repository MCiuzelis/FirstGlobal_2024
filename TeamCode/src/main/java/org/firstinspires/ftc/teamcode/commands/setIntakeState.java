package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class setIntakeState extends SequentialCommandGroup {
    public setIntakeState(IntakeSubsystem intake, IntakeState state) {
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //IF HOLDING BALL
                                new setIntakeSpeedCommand(intake, -1),
                                new WaitCommand(300),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),
                                new WaitCommand(800),
                                new setIntakeSpeedCommand(intake, 0),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
                        ),

                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        //up
                                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                                        new setIntakeSpeedCommand(intake, IntakeSubsystem.nominalSpeed)
                                ),
                                new ParallelCommandGroup(
                                        //down
                                        new setIntakeSpeedCommand(intake, 0),
                                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
                                ),
                                () -> state.equals(IntakeState.UP)

                        ),
                        ()-> intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL)
                )
        );
        addRequirements(intake);
    }

    public enum IntakeState {
        UP,
        DOWN
    }
}

