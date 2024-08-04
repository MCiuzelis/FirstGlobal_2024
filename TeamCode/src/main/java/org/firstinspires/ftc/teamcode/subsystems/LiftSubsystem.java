package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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
        this.telemetry = telemetry;
    }


    public void periodic() {
        controller.setPID(p, i, d);

        double currentPosition = robot.liftPosition.getRotation();
        double power = controller.calculate(currentPosition, targetPosition);
        robot.setLiftPower(power);
    }

    public void setTargetPosition(TARGET_POSITION position){
        switch (position) {
            case BOTTOM:
                targetPosition = lowPosition;
                break;
            case TOP:
                targetPosition = highPosition;
                break;
        }
    }

    public enum TARGET_POSITION{
        BOTTOM,
        TOP
    }
}
