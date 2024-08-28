package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
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
    public static double lowPosition = 60;
    public static double midPosition = 70;
    public static double intakeBlockingPosition = 6;
    public static double initialPosition = 0;

    public static double left_servoHoldPos = 0.52;
    public static double left_servoReleasePos = 0;
    public static double left_servoIntakeBlockingPos = 0.62;
    public static double right_servoHoldPos = 0.58;
    public static double right_servoReleasePos = 0.3;
    public static double right_servoIntakeBlockingPos = 0.685;

    public static double errorMargin = 0.5;

//    public static double distanceTargetWhenDown = 18;
//    public static double distanceTargetWhenBlockingIntake = 20;

    public static double distanceTargetWhenDown = 5;
    public static double distanceTargetWhenBlockingIntake = 5;

    public static double p = 0.8;
    public static double i = 0.0037;
    public static double d = 0.0023;
    public static double f = 0.0038;

    private double targetPosition = 0;
    private final PIDFController controller = new PIDFController(p, i, d, f);

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
        telemetry.addData("lift target position: ", targetPosition);
        telemetry.addData("current lift position: ", currentPosition);
        power = clamp(power, -0.5, 1);
        robot.setLiftPower(power);
    }

    public void setTargetPosition(LIFT_POSITION position){
        switch (position) {
            case INITIAL:
                targetPosition = initialPosition;
                break;
            case DOWN:
            case BLOCKING_INTAKE:
                targetPosition = intakeBlockingPosition;
                break;
            case LOW:
                targetPosition = lowPosition;
                break;
            case MID:
                targetPosition = midPosition;
                break;
            case HIGH:
                targetPosition = highPosition;
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
        double distance = robot.sensor.getDistance();
        switch (currentState){
            case DOWN:
                return distance < distanceTargetWhenDown;
            case BLOCKING_INTAKE:
                return distance < distanceTargetWhenBlockingIntake;
            default:
                throw new RuntimeException("called isBallPresent on a not recognised state");
        }
    }

    public void offsetTarget(double offset){
        targetPosition += offset;
        targetPosition = clamp(targetPosition, lowPosition, highPosition);
    }

    public double getHeight(){
        return robot.encoder_liftPosition.getRotation() / highPosition;
    }

    public double mapValue(double x) {
        double a1 = 6 / 82.5;
        double b1 = 1;
        double a2 = 1;
        double b2 = 0.3;

        x = clamp(x, a1, b1);
        return a2 + ((x - a1) * (b2 - a2) / (b1 - a1));
    }

    public enum LIFT_POSITION {
        INITIAL,
        BLOCKING_INTAKE,
        DOWN,
        LOW,
        MID,
        HIGH,
    }

    public enum SERVO_POSITION{
        RELEASE,
        HOLD,
        BLOCK_INTAKE
    }
}