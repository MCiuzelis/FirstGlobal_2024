package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class DriveTrainSubsystem extends SubsystemBase {
    RobotHardware robot;
    Telemetry telemetry;

    public static double turnOutputCap = 0.5;
    LowPassFilter leftFilter = new LowPassFilter(0.9);
    LowPassFilter rightFilter = new LowPassFilter(0.9);

    public DriveTrainSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop(Pose input){
        input.rotateVector(-90);
        double drive = input.getY();
        double turn = clamp(input.getTurn(), -turnOutputCap, turnOutputCap);

        if (drive != 0) turn *= clamp(Math.abs(drive + Math.signum(drive) * 0.2), 0, 1) * Math.signum(drive);
        else turn *= 1;

        double leftPower = drive - turn;
        double rightPower = drive + turn;
        double[] powers = getScaledPowerOutputs(leftPower, rightPower);

        robot.setLeftPower(leftFilter.estimate(powers[0]));
        robot.setRightPower(rightFilter.estimate(powers[1]));
        telemetry.addData("leftPower(not scaled): ", leftPower);
        telemetry.addData("rightPower(not scaled): ", rightPower);
    }


    private double[] getScaledPowerOutputs(double... powers){
        double maxPower = 0;
        for(int i = 0; i < powers.length; i++){
            double power = Math.abs(powers[i]);
            if (power > maxPower){
                maxPower = power;
            }
        }
        if (maxPower > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
        return powers;
    }
}