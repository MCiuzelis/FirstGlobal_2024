package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double highPosition = 100;
    public static double lowPosition = 0;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;

    private double targetPosition = 0;
    private PIDController controller = new PIDController(p, i, d);

    RobotHardware robot;
    Telemetry telemetry;

    public LiftSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void periodic() {
        controller.setPID(p, i, d);

        double currentPosition = robot.encoder_liftPosition.getRotation();
        double power = controller.calculate(currentPosition, targetPosition);
        robot.setLiftPower(power);
    }

    public void setTargetPosition(TARGET_POSITION position){
        switch (position) {
            case DOWN:
                targetPosition = lowPosition;
                break;
            case UP:
                targetPosition = highPosition;
                break;
        }
    }

    public void offsetTarget(double offset){
        targetPosition += offset;
        targetPosition = clamp(targetPosition, lowPosition, highPosition);
    }

    public enum TARGET_POSITION{
        DOWN,
        UP
    }
}
