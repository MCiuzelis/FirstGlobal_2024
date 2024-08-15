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

    public static double angle_DOWN = 0;
    public static double angle_UP = 0;

    public static double nominalSpeed = 0.75;

    public static double angleOffsetMultiplier = 10;

    private double targetAngle = 0;
    private double targetSpeed = 0;
    private double angleOffset = 0;

    private PIDController anglePID = new PIDController(angle_p, angle_i, angle_d);

    RobotHardware robot;
    Telemetry telemetry;

    public IntakeSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
    }


    public void setAngle(INTAKE_ANGLE angle){
        switch (angle){
            case DOWN:
                targetAngle = angle_DOWN;
                break;
            case UP:
                targetAngle = angle_UP;
                break;
        }
    }

    public void setAngleOffset(double offset){
        if (targetAngle == angle_UP) angleOffset = offset * angleOffsetMultiplier;
        else angleOffset = 0;
    }

    public void setSpeed(double speed){
        targetSpeed = speed;
    }

    public void periodic(){
        anglePID.setPID(angle_p, angle_i, angle_d);

        double currentAngle = Math.toDegrees(robot.encoder_intake_Angle.getRotation());
        double anglePower = anglePID.calculate(currentAngle, targetAngle - angleOffset);

        robot.intake_AngleMotor.setPower(anglePower);
        robot.intake_spinyMotor.setPower(targetSpeed);

        telemetry.addData("currentIntakeAngle: ", currentAngle);
    }


    public enum INTAKE_ANGLE{
        DOWN,
        UP
    }
}
