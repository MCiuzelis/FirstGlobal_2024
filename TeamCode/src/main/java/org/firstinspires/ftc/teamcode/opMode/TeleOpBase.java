package org.firstinspires.ftc.teamcode.opMode;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.BetterGamepad;

public abstract class TeleOpBase extends CommandOpMode {
    public RobotHardware robot;
    public BetterGamepad driver;
    public DriveTrainSubsystem tank;
    private boolean start = false;
    private double loopTime = 0;

    @Override
    public void initialize(){
        robot = new RobotHardware(hardwareMap);
        robot.initialiseHardware(telemetry);

        tank = new DriveTrainSubsystem(robot, telemetry);
        driver = new BetterGamepad(gamepad1);

        Init();
        while (opModeInInit()) InitLoop();
    }

    @Override
    public void run(){
        if (!start){
            Start();
            start = true;
        }
        Loop();

        double loop = System.nanoTime();
        telemetry.addData("Loop time ms: ",  (loop - loopTime) / 1000000);
        loopTime = loop;

        telemetry.update();
        if (isStopRequested()){
            Stop();
            stop();
        }
    }

    public abstract void Init();
    public void InitLoop(){};
    public void Start(){};
    public abstract void Loop();
    public void Stop(){};
}