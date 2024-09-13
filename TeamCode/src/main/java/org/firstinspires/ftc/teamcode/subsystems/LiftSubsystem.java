package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double highPosition = 82.5;
    public static double lowPosition = 65;
    public static double midPosition = 74;
    public static double transferPosition = 25;
    public static double intakeBlockingPosition = 6;
    public static double initialPosition = 0;

    public static double left_servoHoldPos = 0.56;
    public static double left_servoReleasePos = 0.1;
    public static double left_servoIntakeBlockingPos = 0.92;
    public static double right_servoHoldPos = 0.59;
    public static double right_servoReleasePos = 0;
    public static double right_servoIntakeBlockingPos = 0.685;

    public static double topServoHoldPos = 0;
    public static double topServoReleasePos = 0;
    public static double topServoBlockingPos = 0;
    public static double topServoFoldedPos = 0;

    public static double errorMargin = 0.5;

    public static double distanceTargetWhenDown = 5;
    public static double distanceTargetWhenBlockingIntake = 5;

    public BALL_STATE currentBallState = BALL_STATE.NOT_IN_TRANSFER;

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

        double currentPosition = robot.encoder_liftPosition.getPosition();
        double power = controller.calculate(currentPosition, targetPosition);
        telemetry.addData("lift target position: ", targetPosition);
        telemetry.addData("current lift position: ", currentPosition);
        power = clamp(power, -0.75, 1);
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
            case TRANSFER:
                targetPosition = transferPosition;
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

    public void update(BUCKET_SERVO_POSITION position){
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

    public void update(TOP_SERVO_POSITION position){
        switch (position){
            case RELEASE:
                robot.topServoLeft.setPosition(topServoReleasePos);
                robot.topServoRight.setPosition(topServoReleasePos);
                break;
            case HOLD:
                robot.topServoLeft.setPosition(topServoHoldPos);
                robot.topServoRight.setPosition(topServoHoldPos);
                break;
            case BLOCKING:
                robot.topServoLeft.setPosition(topServoBlockingPos);
                robot.topServoRight.setPosition(topServoBlockingPos);
                break;
            case FOLDED:
                robot.topServoLeft.setPosition(topServoFoldedPos);
                robot.topServoRight.setPosition(topServoFoldedPos);
                break;
        }
    }

    public void setBallState(BALL_STATE state){
        currentBallState = state;
    }

    public BALL_STATE getBallState(){
        return currentBallState;
    }

    public boolean liftReachedPosition(){
        return robot.encoder_liftPosition.getPosition() > targetPosition - errorMargin && robot.encoder_liftPosition.getPosition() < targetPosition + errorMargin;
    }

    public boolean areServosExtended(){
        return robot.releaseServoLeft.getPosition() == left_servoHoldPos;
    }

    public boolean isLiftAboveTransferPos(){
        return robot.encoder_liftPosition.getPosition() > transferPosition + errorMargin;
    }

    public boolean isLiftCloseToTransferPos(){
        return robot.encoder_liftPosition.getPosition() < transferPosition + 9;
    }

    public boolean isLiftHighEnoughToFoldIntake(){
        return robot.encoder_liftPosition.getPosition() > 40;
    }

    public boolean isLiftUP(){
        return robot.encoder_liftPosition.getPosition() >= lowPosition;
    }

    public boolean isBallPresent(LIFT_POSITION currentState){
        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);
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
        return robot.encoder_liftPosition.getPosition() / highPosition;
    }

    public double mapValue(double variable, double min_IN, double max_IN, double min_OUT, double max_OUT) {
        variable = clamp(variable, min_IN, max_IN);
        return min_OUT + ((variable - min_IN) * (max_OUT - min_OUT) / (max_IN - min_IN));
    }

    public enum LIFT_POSITION {
        INITIAL,
        BLOCKING_INTAKE,
        DOWN,
        TRANSFER,
        LOW,
        MID,
        HIGH,
    }

    public enum BUCKET_SERVO_POSITION {
        RELEASE,
        HOLD,
        BLOCK_INTAKE
    }

    public enum TOP_SERVO_POSITION {
        HOLD,
        RELEASE,
        BLOCKING,
        FOLDED
    }

    public enum BALL_STATE {
        IN_TRANSFER,
        NOT_IN_TRANSFER
    }
}