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

    public static double f_spin = 0.1;

    public static double p_hold = 0.05;
    public static double i_hold = 4.5;
    public static double d_hold = 0;

    public static double angle_DOWN = 9;
    public static double angleSlowDown = 22;
    public static double angle_UP = 67;
    public static double angle_TRANSFER = 165;
    public static double angle_SPIN_OUT = 55;
    public static double angle_HOLDING_BALL = 45;
    public static double angle_HOLDING_BALL_OUTSIDE = 50;
    public static double angle_FOLDED = 200;

    public static double slowDownSpeed_fast = 2;
    public static double slowDownSpeed_slow = 0.2;
    public static double errorMarin = 1;

    public static double nominalSpeed = 0.95;
    public static double maxAngleOffset = 18;

    public static double spinyMotorCurrentCap = 3300;
    public static double angleMotorCalibrationCurrentCap = 1200;

    public static INTAKE_ANGLE currentState = INTAKE_ANGLE.DOWN;
    public static MOTOR_MODE currentMode = MOTOR_MODE.HOLD_POSITION;

    private double targetAngle = 0;
    private double targetSpeed = 0;
    private double targetSpinyPosition = 0;
    private double angleOffset = 0;
    private boolean goDown = false;

    private final LowPassFilter speedFilter = new LowPassFilter(0.9);
    private final LowPassFilter currentFilter = new LowPassFilter(0.7);

    private final PIDController anglePID = new PIDController(p, i, d);
    private final PIDController holdPID = new PIDController(p_hold, i_hold, d_hold);

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
                currentState = INTAKE_ANGLE.DOWN;
                break;
            case UP:
                goDown = false;
                targetAngle = angle_UP;
                currentState = INTAKE_ANGLE.UP;
                break;
            case TRANSFER:
                goDown = false;
                targetAngle = angle_TRANSFER;
                currentState = INTAKE_ANGLE.TRANSFER;
                break;
            case HOLDING_BALL_INSIDE:
                goDown = false;
                targetAngle = angle_HOLDING_BALL;
                currentState = INTAKE_ANGLE.HOLDING_BALL_INSIDE;
                break;
            case HOLDING_BALL_OUTSIDE:
                goDown = false;
                targetAngle = angle_HOLDING_BALL_OUTSIDE;
                currentState = INTAKE_ANGLE.HOLDING_BALL_INSIDE;
                break;
            case SPIN_OUT:
                goDown = false;
                targetAngle = angle_SPIN_OUT;
                currentState = INTAKE_ANGLE.UP;
                break;
            case FOLDED:
                goDown = false;
                targetAngle = angle_FOLDED;
                currentState = INTAKE_ANGLE.FOLDED;
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
        holdPID.setPID(p_hold, i_hold, d_hold);

        double currentAngle = getAngleDegrees(robot.encoder_intake_Angle.getPosition());

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

        if (currentAngle <= angle_DOWN + errorMarin) goDown = false;
        targetAngle = clamp(targetAngle, angle_DOWN, angle_FOLDED);

        double anglePower;
        if (currentState != INTAKE_ANGLE.HOLDING_BALL_INSIDE && currentState != INTAKE_ANGLE.HOLDING_BALL_OUTSIDE) {
            anglePower = anglePID.calculate(currentAngle, targetAngle - angleOffset) + Math.sin(Math.toRadians(currentAngle)) * f;
        }
        else{
            anglePower = anglePID.calculate(currentAngle, targetAngle) + Math.sin(Math.toRadians(currentAngle)) * f;
        }

        if (currentAngle <= angle_DOWN + errorMarin) {
            if (targetAngle != angle_TRANSFER && targetAngle != angle_UP) anglePower = 0;
        }

        robot.intake_AngleMotor.setPower(anglePower);
        double angleMotorVelocity = robot.encoder_intake_Angle.getVelocity();
        double spinMotorFeedForward = angleMotorVelocity * f_spin;

        switch (currentMode){
            case SET_VELOCITY:
                robot.intake_spinyMotor.setPower(spinMotorFeedForward + speedFilter.estimate(targetSpeed));
                break;
            case HOLD_POSITION:
                double power = holdPID.calculate(robot.encoder_intake_Speed.getVelocity(), angleMotorVelocity);
                robot.intake_spinyMotor.setPower(spinMotorFeedForward + power);
                break;
        }


        telemetry.addData("intakeAnglePower: ", anglePower);
        telemetry.addData("intake angle: ", currentAngle);
        telemetry.addData("targetAngle: ", targetAngle);
        telemetry.addData("angleOffset: ", angleOffset);
        telemetry.addData("intake angle Velocity: ", robot.encoder_intake_Speed.getVelocity());
        telemetry.addData("intake speed Velocity: ", robot.encoder_intake_Speed.getVelocity());
    }

    public void setMode(MOTOR_MODE mode){
        currentMode = mode;
    }

    public void setMode(MOTOR_MODE mode, double velocity){
        setMode(mode);
        setSpeed(velocity);
    }

    public void calibrateAngle() {
        while(robot.intake_AngleMotor.getCurrent() < angleMotorCalibrationCurrentCap){
            robot.intake_AngleMotor.setPower(-0.5);
        }
        robot.intake_AngleMotor.setPower(0);
        robot.encoder_intake_Angle.reset();
    }

    public boolean overCurrentTriggered(double speed) {
        double current = robot.intake_spinyMotor.getCurrent();
        telemetry.addData("spiny motor current: ", current);
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

    public enum MOTOR_MODE {
        HOLD_POSITION,
        SET_VELOCITY
    }
}