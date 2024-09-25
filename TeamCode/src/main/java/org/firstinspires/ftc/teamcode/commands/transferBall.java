package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setTopServoState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterGamepad;

public class transferBall extends SequentialCommandGroup {
    public transferBall(IntakeSubsystem intake, LiftSubsystem lift, BetterGamepad driver){
        addCommands(
                new InstantCommand(()-> lift.setBallState(LiftSubsystem.BALL_STATE.NOT_IN_TRANSFER)),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.SPIN_OUT),

                new WaitUntilCommand(intake::intakeNotFolded),

                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.INITIAL),
                new WaitUntilCommand(lift::liftReachedPosition),
                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.RELEASE),

                new WaitCommand(300),

                new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.DOWN),
                new setIntakeSpeedCommand(intake, -1),
                new WaitCommand(250),

//                new ParallelRaceGroup(
//                    new WaitUntilCommand(()-> lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)),
//                    new WaitCommand(4000)
//                ),

                new ParallelRaceGroup(
                        new WaitUntilCommand(()-> lift.isBallPresent(LiftSubsystem.LIFT_POSITION.BLOCKING_INTAKE)),
                        new WaitUntilCommand(()-> driver.getGamepadButton(driver.share).get())
                ),


                new setIntakeSpeedCommand(intake, 0),
                new WaitCommand(100),

                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),

                new WaitCommand(400),

                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW),

                new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.RELEASE),
                new WaitCommand(50),
                new setTopServoState(lift, LiftSubsystem.TOP_SERVO_POSITION.FOLDED),

                new WaitCommand(100),
                new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW)
        );
        addRequirements(lift, intake);
    }
}
