package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.rumbleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterGamepad;

public class intakeBall extends SequentialCommandGroup {
    public intakeBall(LiftSubsystem lift, IntakeSubsystem intake, BetterGamepad gamepad){
        addCommands(
                new ConditionalCommand(
                        //if LIFT UP, rumble
                        new rumbleCommand(gamepad, 100),

                        //if LIFT DOWN
                        new SequentialCommandGroup(
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.INITIAL),
                                new WaitUntilCommand(lift::liftReachedPosition),
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                new WaitCommand(100),

                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                                new setIntakeSpeedCommand(intake, 0.85),

                                new ParallelRaceGroup(
                                        new WaitUntilCommand(()-> lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)),
                                        new WaitCommand(5000)
                                ),

                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                new WaitCommand(100),
                                new setIntakeSpeedCommand(intake, 0),

                                new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new rumbleCommand(gamepad, 100),
                                                    new setIntakeSpeedCommand(intake, -1),
                                                    new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                                                    new WaitCommand(1000),
                                                    new setIntakeSpeedCommand(intake, 0),
                                                    new WaitCommand(500),
                                                    new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                                    new WaitCommand(500)
                                        ),
                                        new emptyCommand(),
                                        intake::angleMotorStalling
                                ),

                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                                new WaitCommand(1100),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN)
                        ),
                        ()-> (lift.isLiftUP() || lift.isBallPresent(10))
                )
        );
        addRequirements(lift, intake);
    }
}
