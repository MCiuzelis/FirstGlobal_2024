package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@Config
public class RobotHardware {
    Telemetry telemetry;
    HardwareMap hw;

    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public CuttleMotor frontLeft, frontRight, backLeft, backRight, liftMotor_Left, liftMotor_Right, intake_AngleMotor, intake_spinyMotor;
    public CuttleEncoder encoder_liftPosition, encoder_intake_Angle, encoder_driveBaseLeft, encoder_driveBaseRight;
    List<LynxModule> allHubs;

    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }


    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hw.getAll(LynxModule.class);
        controlHub = new CuttleRevHub(hw, CuttleRevHub.HubTypes.CONTROL_HUB);
        expansionHub = new CuttleRevHub(hw, CuttleRevHub.HubTypes.EXPANSION_HUB);

        frontLeft = initMotor(controlHub, 2, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = initMotor(controlHub, 1, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight = initMotor(controlHub, 0, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = initMotor(controlHub, 3, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor_Left = initMotor(expansionHub, 0, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor_Right = initMotor(expansionHub, 1, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        intake_AngleMotor = initMotor(expansionHub, 2, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        intake_spinyMotor = initMotor(expansionHub, 3, Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        encoder_liftPosition = new CuttleEncoder(controlHub, 1, 530.05128205128);
        encoder_intake_Angle = new CuttleEncoder(controlHub, 2, 288d * 15 / 40);
        encoder_driveBaseLeft = new CuttleEncoder(controlHub, 3, 0);
        encoder_driveBaseRight = new CuttleEncoder(controlHub, 0, 0);
    }

    public void setLeftPower(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void setRightPower(double power){
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void setLiftPower(double power){
        liftMotor_Left.setPower(power);
        liftMotor_Right.setPower(power);
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
}