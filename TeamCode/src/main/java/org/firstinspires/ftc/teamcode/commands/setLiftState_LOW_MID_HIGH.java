package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.wrappers.emptyCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setFrontServoState;
import org.firstinspires.ftc.teamcode.commands.wrappers.setIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.commands.wrappers.setLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setLiftState_LOW_MID_HIGH extends CommandBase {
    private STATE state;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    public setLiftState_LOW_MID_HIGH(IntakeSubsystem intake, LiftSubsystem lift, STATE state){
        this.intake = intake;
        this.lift = lift;
        this.state = state;

        addRequirements(intake, lift);
    }

    public void initialize(){
        SequentialCommandGroup mainCommand = new SequentialCommandGroup(
                new setFrontServoState(lift, LiftSubsystem.BUCKET_SERVO_POSITION.HOLD),

                new WaitUntilCommand(lift::isLiftHighEnoughToFoldIntake),
                new ConditionalCommand(
                        new setIntakeAngleCommand(intake, IntakeSubsystem.INTAKE_ANGLE.FOLDED),
                        new emptyCommand(),
                        ()-> lift.getBallState().equals(LiftSubsystem.BALL_STATE.IN_TRANSFER)
        ));

        switch (state){
            case LOW:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.LOW), mainCommand));
                break;
            case MID:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.MID), mainCommand));
                break;
            case HIGH:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new setLiftHeightCommand(lift, LiftSubsystem.LIFT_POSITION.HIGH), mainCommand));
                break;
        }
    }

    public enum STATE{
        LOW,
        MID,
        HIGH
    }
}