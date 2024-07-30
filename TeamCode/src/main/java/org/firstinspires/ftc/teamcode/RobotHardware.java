package org.firstinspires.ftc.teamcode;

import androidx.annotation.GuardedBy;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;


@Config
public class RobotHardware {
    Telemetry telemetry;
    HardwareMap hw;

    public CuttleRevHub controlHub;
    public CuttleMotor frontLeft, frontRight, backLeft, backRight;

    List<LynxModule> allHubs;

    private final Object imuLock = new Object();

    @GuardedBy("imuLock")
    private IMU imu;

    //private RevIMU imu;

    public Rotation2d imuAngle = new Rotation2d();
    private Thread imuThread;


    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }




    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hw.getAll(LynxModule.class);

        controlHub = new CuttleRevHub(hw, CuttleRevHub.HubTypes.CONTROL_HUB);

        frontLeft = initMotor(controlHub, 2, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = initMotor(controlHub, 1, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight = initMotor(controlHub, 0, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = initMotor(controlHub, 3, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        synchronized (imuLock) {
            //imu = new RevIMU(hw, "imu");

            imu = hw.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
            //imu.init();
            //imu.reset();
        }
    }

    public void setLeftPower(double power){
        frontLeft.setPower(power * 1.1);
        backLeft.setPower(power * 1.1);
    }

    public void setRightPower(double power){
        frontRight.setPower(power);    //TODO FIX LATER
        backRight.setPower(power);
    }





    public void setBulkCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    private CuttleMotor initMotor(CuttleRevHub hub, int motorPort, Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        CuttleMotor motor = new CuttleMotor(hub, motorPort);
        motor.setDirection(direction);
        motor.setZeroPowerBehaviour(zeroPowerBehavior);
        return motor;
    }

    private Servo initServo (HardwareMap hw, String servoPort, Servo.Direction direction){
        Servo servo = hw.get(Servo.class, servoPort);
        servo.setDirection(direction);
        return servo;
    }

    public void startIMUThread(LinearOpMode opMode){
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                }
            }
        });
        imuThread.start();
    }

    public void updateIMUStupidMonkeyMethod(){
        imuAngle = new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}