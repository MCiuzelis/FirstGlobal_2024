package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class depositBall extends SequentialCommandGroup {
    public depositBall(IntakeSubsystem intake, LiftSubsystem lift){
        addCommands(
                        new ConditionalCommand(
                                //IF HOLDING BALL
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                //IF BALL INSIDE
                                                new setIntakeSpeedCommand(intake, -1),
                                                new WaitCommand(300),
                                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),
                                                new WaitCommand(800),
                                                new setIntakeSpeedCommand(intake, 0),
                                                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD)
                                        ),

                                        new SequentialCommandGroup(
                                                    //IF BALL OUTSIDE
                                                    new setIntakeSpeedCommand(intake, 1),
                                                    new WaitCommand(600),
                                                    new setIntakeSpeedCommand(intake, 0),
                                                    new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN)
                                                ),
                                        ()-> intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE)
                                ),
                                new SequentialCommandGroup(
                                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.INITIAL),
                                        new WaitUntilCommand(lift::liftReachedPosition),
                                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),
                                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.UP),
                                        new setIntakeSpeedCommand(intake, -1),
                                        new WaitUntilCommand(()-> !lift.isBallPresent(9)),
                                        new WaitCommand(800),
                                        new setIntakeSpeedCommand(intake, 0),
                                        new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),
                                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                                        new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.DOWN)
                                        ),
                                ()-> (intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_INSIDE) ||
                                        intake.getCurrentState().equals(IntakeSubsystem.INTAKE_ANGLE.HOLDING_BALL_OUTSIDE)
                        )
        ));
        addRequirements(intake, lift);
    }
}