package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class swallowBall_Inside extends SequentialCommandGroup {
    public swallowBall_Inside(LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE),
                new ConditionalCommand(
                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.BLOCK_INTAKE),
                        ()-> lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)
                ),
                new WaitUntilCommand(lift::liftReachedPosition),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                new setIntakeSpeedCommand(intake, 0.7),

                new WaitCommand(200),
                new WaitUntilCommand(()-> intake.overCurrentTriggered(0.6)),
                new InstantCommand(()->intake.setAngle(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE)),
                new setIntakeSpeedCommand(intake, 0)
        );
        addRequirements(lift, intake);
    }
}
