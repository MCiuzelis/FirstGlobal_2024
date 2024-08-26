package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.utils.BetterGamepad;

public class intakeBall extends SequentialCommandGroup {
    public intakeBall(LiftSubsystem lift, IntakeSubsystem intake, BetterGamepad gamepad){
        addCommands(
                new ConditionalCommand(
                        //if LIFT UP, rumble
                        new rumbleCommand(gamepad, 100),

                        //if LIFT DOWN
                        new SequentialCommandGroup(
                                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.INITIAL),
                                new WaitUntilCommand(lift::liftReachedPosition),
                                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.RELEASE),
                                new WaitCommand(300),
                                new setIntakeState(intake, setIntakeState.IntakeState.UP),
                                new WaitUntilCommand(()-> lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)),
                                new setFrontServoState(lift, LiftSubsystem.SERVO_POSITION.HOLD),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN),
                                new setIntakeState(intake, setIntakeState.IntakeState.DOWN)
                        ),
                        lift::isLiftUP
                )
        );
        addRequirements(lift, intake);
    }
}
