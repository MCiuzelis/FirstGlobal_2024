package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterEncoder;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.utils.wrappers.BetterServo;

@Config
public class RobotHardware {
    Telemetry telemetry;
    HardwareMap hw;

    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public BetterMotor frontLeft, frontRight, backLeft, backRight, liftMotor_Left, liftMotor_Right, intake_AngleMotor, intake_spinyMotor;
    public BetterEncoder encoder_liftPosition, encoder_intake_Angle, encoder_intake_Speed;
    public BetterServo releaseServoLeft, releaseServoRight, topServoLeft, topServoRight;
    public RevColorSensorV3 colorSensor;

    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }


    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controlHub = new CuttleRevHub(hw, CuttleRevHub.HubTypes.CONTROL_HUB);
        controlHub.setI2CBusSpeed(CuttleRevHub.I2CSpeed.HIGH_SPEED);

        expansionHub = new CuttleRevHub(hw, "Expansion Hub 2");

        frontLeft = initMotor(controlHub, 2, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = initMotor(controlHub, 3, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight = initMotor(controlHub, 1, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = initMotor(controlHub, 0, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor_Left = initMotor(expansionHub, 0, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor_Right = initMotor(expansionHub, 1, Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        intake_AngleMotor = initMotor(expansionHub, 3, Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        intake_spinyMotor = initMotor(expansionHub, 2, Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        encoder_liftPosition = new BetterEncoder(controlHub, 3, 530.05128205128);
        encoder_liftPosition.setDirection(Direction.REVERSE);
        encoder_intake_Angle = new BetterEncoder(controlHub, 0, 288d * 40 / 15);
        encoder_intake_Angle.setDirection(Direction.REVERSE);

        releaseServoLeft = new BetterServo(controlHub, 0, BetterServo.Direction.FORWARD);
        releaseServoRight = new BetterServo(controlHub, 1, BetterServo.Direction.REVERSE);

        topServoLeft = new BetterServo(controlHub, 2, BetterServo.Direction.FORWARD);
        topServoRight = new BetterServo(controlHub, 3, BetterServo.Direction.REVERSE);

        colorSensor = hw.get(RevColorSensorV3.class, "colorSensor");
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

    private BetterMotor initMotor(CuttleRevHub hub, int motorPort, Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        BetterMotor motor = new BetterMotor(hub, motorPort);
        motor.setDirection(direction);
        motor.setZeroPowerBehaviour(zeroPowerBehavior);
        return motor;
    }
}