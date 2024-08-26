package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double highPosition = 82.5;
    public static double lowPosition = 3;
    public static double intakeBlockingPosition = 6;
    public static double initialPosition = 0;

    public static double left_servoHoldPos = 0.52;
    public static double left_servoReleasePos = 0;
    public static double left_servoIntakeBlockingPos = 0.62;
    public static double right_servoHoldPos = 0.58;
    public static double right_servoReleasePos = 0.3;
    public static double right_servoIntakeBlockingPos = 0.685;

    public static double errorMargin = 0.1;

    public static double distanceTargetWhenDown = 18;
    public static double distanceTargetWhenBlockingIntake = 20;
    public static double distanceTargetWhenUP = 120;

    public static double p = 0.7;
    public static double i = 0.0037;
    public static double d = 0.0023;
    public static double f = 0.0038;

    private double targetPosition = 0;
    private PIDFController controller = new PIDFController(p, i, d, f);

    RobotHardware robot;
    Telemetry telemetry;

    public LiftSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void periodic() {
        controller.setPIDF(p, i, d, f);

        double currentPosition = robot.encoder_liftPosition.getRotation();
        double power = controller.calculate(currentPosition, targetPosition);
        power = clamp(power, -0.5, 0.5);
        robot.setLiftPower(power);
    }

    public void setTargetPosition(LIFT_POSITION position){
        switch (position) {
            case DOWN:
                targetPosition = lowPosition;
                break;
            case UP:
                targetPosition = highPosition;
                break;
            case BLOCKING_INTAKE:
                targetPosition = intakeBlockingPosition;
                break;
            case INITIAL:
                targetPosition = initialPosition;
                break;
        }
    }

    public void update(SERVO_POSITION position){
        switch (position) {
            case RELEASE:
                robot.releaseServoLeft.setPosition(left_servoReleasePos);
                robot.releaseServoRight.setPosition(right_servoReleasePos);
                break;
            case HOLD:
                robot.releaseServoLeft.setPosition(left_servoHoldPos);
                robot.releaseServoRight.setPosition(right_servoHoldPos);
                break;
            case BLOCK_INTAKE:
                robot.releaseServoLeft.setPosition(left_servoIntakeBlockingPos);
                robot.releaseServoRight.setPosition(right_servoIntakeBlockingPos);
                break;
        }
    }

    public boolean liftReachedPosition(){
        return robot.encoder_liftPosition.getRotation() > targetPosition - errorMargin && robot.encoder_liftPosition.getRotation() < targetPosition + errorMargin;
    }

    public boolean isLiftUP(){
        return targetPosition == highPosition;
    }

    public boolean isBallPresent(LIFT_POSITION currentState){
        switch (currentState){
            case DOWN:
                return robot.distanceSensor.getDistance() < distanceTargetWhenDown;
            case BLOCKING_INTAKE:
                return robot.distanceSensor.getDistance() < distanceTargetWhenBlockingIntake;
            case UP:
                return robot.distanceSensor.getDistance() < distanceTargetWhenUP;
            default:
                throw new RuntimeException("called isBallPresent on a not recognised state");
        }
    }

    public void offsetTarget(double offset){
        targetPosition += offset;
        targetPosition = clamp(targetPosition, lowPosition, highPosition);
    }

    public double getHeight(){
        return clamp(1.2 - (robot.encoder_liftPosition.getRotation() / highPosition), 0 ,1);
    }

    public enum LIFT_POSITION {
        DOWN,
        UP,
        BLOCKING_INTAKE,
        INITIAL
    }
    public enum SERVO_POSITION{
        RELEASE,
        HOLD,
        BLOCK_INTAKE
    }
}