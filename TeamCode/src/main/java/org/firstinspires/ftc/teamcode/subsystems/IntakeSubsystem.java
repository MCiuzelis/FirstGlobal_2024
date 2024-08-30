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
    public static double angle_UP = 67;
    public static double angle_SPIN_OUT = 55;
    public static double angle_HOLDING_BALL = 45;

    public static double slowDownSpeed_fast = 4;
    public static double slowDownSpeed_slow = 0.3;
    public static double errorMarin = 1;

    public static double nominalSpeed = 0.95;
    public static double maxAngleOffset = 18;

    public static double spinyMotorCurrentCap = 3250;
    public static double angleMotorCalibrationCurrentCap = 1200;
    public static INTAKE_ANGLE currentState = INTAKE_ANGLE.DOWN;

    private double targetAngle = 0;
    private double targetSpeed = 0;
    private double angleOffset = 0;
    private boolean goDown = false;

    private final LowPassFilter speedFilter = new LowPassFilter(0.9);
    private final PIDController anglePID = new PIDController(p, i, d);

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
            case HOLDING_BALL:
                goDown = false;
                targetAngle = angle_HOLDING_BALL;
                currentState = INTAKE_ANGLE.HOLDING_BALL;
                break;
            case SPIN_OUT:
                goDown = false;
                targetAngle = angle_SPIN_OUT;
                currentState = INTAKE_ANGLE.UP;
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

        if (currentAngle <= angle_DOWN + errorMarin) goDown = false;
        targetAngle = clamp(targetAngle, angle_DOWN, angle_UP);

        double anglePower;
        if (currentState != INTAKE_ANGLE.HOLDING_BALL) {
            anglePower = anglePID.calculate(currentAngle, targetAngle - angleOffset) + Math.sin(Math.toRadians(currentAngle)) * f;
        }
        else{
            anglePower = anglePID.calculate(currentAngle, targetAngle) + Math.sin(Math.toRadians(currentAngle)) * f;
        }
        if (currentAngle <= angle_DOWN + errorMarin && targetAngle != angle_UP) anglePower = 0;

        robot.intake_AngleMotor.setPower(anglePower);
        robot.intake_spinyMotor.setPower(speedFilter.estimate(targetSpeed));

        telemetry.addData("intakeAnglePower: ", anglePower);
        telemetry.addData("intake angle: ", currentAngle);
        telemetry.addData("targetAngle: ", targetAngle);
        telemetry.addData("angleOffset: ", angleOffset);
    }

    public void calibrateAngle() {
        while(robot.intake_AngleMotor.getCurrent() < angleMotorCalibrationCurrentCap){
            robot.intake_AngleMotor.setPower(-0.5);
        }
        robot.intake_AngleMotor.setPower(0);
        robot.encoder_intake_Angle.reset();
    }

    public boolean overCurrentTriggered() {
        double current = robot.intake_spinyMotor.getCurrent();
        telemetry.addData("spiny motor current: ", current);
        return current > spinyMotorCurrentCap;
    }

    private double getAngleDegrees(){
        return Math.toDegrees(robot.encoder_intake_Angle.getPosition()) + angle_DOWN;
    }

    public INTAKE_ANGLE getCurrentState(){
        return currentState;
    }

    public enum INTAKE_ANGLE{
        DOWN,
        UP,
        SPIN_OUT,
        HOLDING_BALL
    }
}