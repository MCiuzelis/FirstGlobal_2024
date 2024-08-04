package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double angle_p = 0;
    public static double angle_i = 0;
    public static double angle_d = 0;

    public static double velocity_p = 0;
    public static double velocity_i = 0;
    public static double velocity_d = 0;

    public static double startingPosition = 0;
    public static double preparedPosition = 0;
    public static double pickupPosition = 0;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    private PIDController anglePID = new PIDController(angle_p, angle_i, angle_d);
    private PIDController velocityPID = new PIDController(velocity_p, velocity_i, velocity_d);

    RobotHardware robot;
    Telemetry telemetry;

    public IntakeSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
    }


    public void setIntakeAngle(INTAKE_ANGLE angle){
        switch (angle){
            case STARTING_POSITION:
                targetAngle = startingPosition;
                break;
            case PREPARED_POSITION:
                targetAngle = preparedPosition;
                break;
            case PICKUP_POSITION:
                targetAngle = pickupPosition;
                break;
        }
    }

    public void setIntakeTargetVelocity(double velocity){
        targetVelocity = velocity;
    }

    public void periodic(){
        anglePID.setPID(angle_p, angle_i, angle_d);
        velocityPID.setPID(velocity_p, velocity_i, velocity_d);

        double currentAngle = robot.intake_Angle.getRotation();
        double currentVelocity = robot.intakeSpeed.getVelocity();

        double anglePower = anglePID.calculate(currentAngle, targetAngle);
        double velocityPower = velocityPID.calculate(currentVelocity, targetVelocity);

        robot.intake_AngleMotor.setPower(anglePower);
        robot.intake_spinyMotor.setPower(velocityPower);

        telemetry.addData("currentIntakeAngle: ", currentAngle);
        telemetry.addData("currentIntakeVelocity: ", currentVelocity);
    }


    public enum INTAKE_ANGLE{
        STARTING_POSITION,
        PREPARED_POSITION,
        PICKUP_POSITION
    }
}
