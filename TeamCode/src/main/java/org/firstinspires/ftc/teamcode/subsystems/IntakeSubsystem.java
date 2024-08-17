package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double p = 0.115;
    public static double i = 0.001;
    public static double d = 0.002;
    public static double f = 0.48;

    public static double angle_DOWN = 9;
    public static double angleSlowDown = 22;
    public static double angle_UP = 70;

    public static double slowDownSpeed_fast = 2.25;
    public static double slowDownSpeed_slow = 0.13;
    public static double errorMarin = 1;

    public static double nominalSpeed = 0.825;
    public static double maxAngleOffset = 15;

    private double targetAngle = 0;
    private double targetSpeed = 0;
    private double angleOffset = 0;
    private boolean goDown = false;

    private PIDController anglePID = new PIDController(p, i, d);

    RobotHardware robot;
    Telemetry telemetry;

    public IntakeSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void setAngle(INTAKE_ANGLE angle){
        switch (angle){
            case DOWN:
                goDown = true;
                break;
            case UP:
                goDown = false;
                targetAngle = angle_UP;
                break;
        }
    }

    public void setAngleOffset(double offset){
        if (!goDown) angleOffset = offset * maxAngleOffset;
        else angleOffset = 0;
    }

    public void setSpeed(double speed){
        targetSpeed = speed;
    }

    public void periodic(){
        anglePID.setPID(p, i, d);

        double currentAngle = getAngleDegrees();

        if (goDown){
            if (currentAngle > angle_DOWN){
                if (currentAngle > angleSlowDown) {
                    targetAngle -= slowDownSpeed_fast;
                }
                else{
                    targetAngle -= slowDownSpeed_slow;
                }
            }
        }

        if (currentAngle <= angle_DOWN + errorMarin){
            goDown = false;
        }

        targetAngle = clamp(targetAngle, angle_DOWN, angle_UP);
        double anglePower = anglePID.calculate(currentAngle, targetAngle - angleOffset) + Math.sin(Math.toRadians(currentAngle)) * f;
        if (currentAngle <= angle_DOWN + errorMarin && targetAngle != angle_UP){
            anglePower = 0;
        }

        robot.intake_AngleMotor.setPower(anglePower);
        robot.intake_spinyMotor.setPower(targetSpeed);

        telemetry.addData("currentAngle: ", currentAngle);
        telemetry.addData("intakeAnglePower: ", anglePower);
        telemetry.addData("sensor reading: ", robot.encoder_intake_Angle.getRotation());
        telemetry.addData("targetAngle: ", targetAngle);
        telemetry.addData("angleOffset: ", angleOffset);
    }

    private double getAngleDegrees(){
        return Math.toDegrees(robot.encoder_intake_Angle.getRotation()) + angle_DOWN;
    }


    public enum INTAKE_ANGLE{
        DOWN,
        UP
    }
}