package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.outoftheboxrobotics.photoncore.Photon;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterGamepad;

@Photon
public abstract class TeleOpBase extends CommandOpMode {
    public BetterGamepad driver;
    public BetterGamepad assistant;

    public RobotHardware robot;
    public DriveTrainSubsystem tank;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    private boolean start = false;
    private double loopTime = 0;

    @Override
    public void initialize(){
        robot = new RobotHardware(hardwareMap);
        robot.initialiseHardware(telemetry);

        tank = new DriveTrainSubsystem(robot, telemetry);
        intake = new IntakeSubsystem(robot, telemetry);
        lift = new LiftSubsystem(robot, telemetry);

        driver = new BetterGamepad(gamepad1);
        assistant = new BetterGamepad(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
    public void InitLoop(){}
    public void Start(){}
    public abstract void Loop();
    public void Stop(){}
}