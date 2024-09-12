package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setTopServoState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class transferBall extends SequentialCommandGroup {
    public transferBall(LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new setIntakeSpeedCommand(intake, 0),
                new setIntakeModeCommand(intake, IntakeSubsystem.MOTOR_MODE.HOLD_POSITION),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.TRANSFER),
                                new WaitUntilCommand(lift::isLiftCloseToTransferPos),
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),
                                new WaitUntilCommand(lift::liftReachedPosition),
                                new WaitCommand(50)
                        ),
                        new SequentialCommandGroup(
                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.TRANSFER),
                                new WaitUntilCommand(lift::isLiftCloseToTransferPos),
                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT)
                        ),
                        lift::isLiftAboveTransferPos
                ),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.TRANSFER),
                new WaitCommand(400),
                new setIntakeSpeedCommand(intake, 0.5),
                new WaitCommand(70),

                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                new WaitCommand(30),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                new setIntakeSpeedCommand(intake, 0),

                new InstantCommand(()-> lift.setBallState(LiftSubsystem.BALL_STATE.NOT_IN_TRANSFER)),

                new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.RELEASE),
                new WaitCommand(50),
                new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.FOLDED)
        );
        addRequirements(lift, intake);
    }
}
