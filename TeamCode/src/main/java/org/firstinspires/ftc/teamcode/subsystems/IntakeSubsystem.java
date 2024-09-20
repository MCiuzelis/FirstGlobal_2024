package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.MotionProfile;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public static double angle_DOWN = 10.25;
    public static double angle_UP = 65;
    public static double angle_TRANSFER = 165;
    public static double angle_SPIN_OUT = 58;
    public static double angle_HOLDING_BALL = 45;
    public static double angle_HOLDING_BALL_OUTSIDE = 50;
    public static double angle_FOLDED = 200;

    public static double errorMarin = 3;
    public static double maxAngleOffset = 25;

    public static double spinyMotorCurrentCap = 3300;
    public static double angleMotorCalibrationCurrentCap = 1200;

    public static INTAKE_ANGLE currentState = INTAKE_ANGLE.DOWN;

    private double targetSpeed = 0;
    private double angleOffset = 0;

    private final LowPassFilter speedFilter = new LowPassFilter(0.9);
    private final LowPassFilter currentFilter = new LowPassFilter(0.7);

    RobotHardware robot;
    Telemetry telemetry;
    MotionProfile motionProfile;

    public IntakeSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionProfile = new MotionProfile(this.telemetry);
    }

    public void setAngle(INTAKE_ANGLE angle){
        double currentAngle = getAngleDegrees(robot.encoder_intake_Angle.getPosition());

        switch (angle){
            case DOWN:
                motionProfile.setTargetPosition(angle_DOWN, currentAngle);
                currentState = INTAKE_ANGLE.DOWN;
                break;
            case UP:
                motionProfile.setTargetPosition(angle_UP, currentAngle);
                currentState = INTAKE_ANGLE.UP;
                break;
            case TRANSFER:
                motionProfile.setTargetPosition(angle_TRANSFER, currentAngle);
                currentState = INTAKE_ANGLE.TRANSFER;
                break;
            case HOLDING_BALL_INSIDE:
                motionProfile.setTargetPosition(angle_HOLDING_BALL, currentAngle);
                currentState = INTAKE_ANGLE.HOLDING_BALL_INSIDE;
                break;
            case HOLDING_BALL_OUTSIDE:
                motionProfile.setTargetPosition(angle_HOLDING_BALL_OUTSIDE, currentAngle);
                currentState = INTAKE_ANGLE.HOLDING_BALL_OUTSIDE;
                break;
            case SPIN_OUT:
                motionProfile.setTargetPosition(angle_SPIN_OUT, currentAngle);
                currentState = INTAKE_ANGLE.UP;
                break;
            case FOLDED:
                motionProfile.setTargetPosition(angle_FOLDED, currentAngle);
                currentState = INTAKE_ANGLE.FOLDED;
                break;
        }
    }

    public void setAngleOffset(double offset){
        angleOffset = offset * maxAngleOffset;
    }

    public void setSpeed(double speed){
        targetSpeed = speed;
    }

    public void periodic(){
        double currentAngle = getAngleDegrees(robot.encoder_intake_Angle.getPosition());

        if (currentState != INTAKE_ANGLE.HOLDING_BALL_INSIDE && currentState != INTAKE_ANGLE.HOLDING_BALL_OUTSIDE) {
            currentAngle += angleOffset;
        }

        robot.intake_AngleMotor.setPower(motionProfile.getPower(currentAngle));
        robot.intake_spinyMotor.setPower(speedFilter.estimate(targetSpeed));

        telemetry.addData("intake angle: ", currentAngle);
        telemetry.addData("angleOffset: ", angleOffset);
    }


    public void calibrateAngle() {
        while(robot.intake_AngleMotor.getCurrent() < angleMotorCalibrationCurrentCap){
            robot.intake_AngleMotor.setPower(-0.5);
        }
        robot.intake_AngleMotor.setPower(0);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.encoder_intake_Angle.reset();
    }

    public boolean angleMotorStalling(){
        return robot.intake_AngleMotor.getCurrent() > 5000;
    }

    public boolean intakeNotFolded(){
        return robot.encoder_intake_Angle.getPosition() < 150;
    }

    public boolean overCurrentTriggered(double speed) {
        double current = robot.intake_spinyMotor.getCurrent();
        return currentFilter.estimate(current) > spinyMotorCurrentCap * Math.abs(speed);
    }

    private double getAngleDegrees(double encoder){
        return Math.toDegrees(encoder) + angle_DOWN;
    }

    public INTAKE_ANGLE getCurrentState(){
        return currentState;
    }

    public enum INTAKE_ANGLE{
        DOWN,
        UP,
        TRANSFER,
        SPIN_OUT,
        HOLDING_BALL_INSIDE,
        HOLDING_BALL_OUTSIDE,
        FOLDED
    }
}