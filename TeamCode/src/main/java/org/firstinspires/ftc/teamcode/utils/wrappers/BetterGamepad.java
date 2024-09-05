package org.firstinspires.ftc.teamcode.utils.wrappers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Pose;

public class BetterGamepad extends GamepadEx {
    public GamepadKeys.Button circle = GamepadKeys.Button.B;
    public GamepadKeys.Button cross = GamepadKeys.Button.A;
    public GamepadKeys.Button triangle = GamepadKeys.Button.Y;
    public GamepadKeys.Button square = GamepadKeys.Button.X;
    public GamepadKeys.Button share = GamepadKeys.Button.BACK;
    public GamepadKeys.Button options = GamepadKeys.Button.START;
    public GamepadKeys.Button dpadUp = GamepadKeys.Button.DPAD_UP;
    public GamepadKeys.Button dpadDown = GamepadKeys.Button.DPAD_DOWN;
    public GamepadKeys.Button dpadLeft = GamepadKeys.Button.DPAD_LEFT;
    public GamepadKeys.Button dpadRight = GamepadKeys.Button.DPAD_RIGHT;
    public GamepadKeys.Button leftBumper = GamepadKeys.Button.LEFT_BUMPER;
    public GamepadKeys.Button rightBumper = GamepadKeys.Button.RIGHT_BUMPER;
    public GamepadKeys.Button M2 = GamepadKeys.Button.LEFT_STICK_BUTTON;
    public GamepadKeys.Button M1 = GamepadKeys.Button.RIGHT_STICK_BUTTON;


    public BetterGamepad(Gamepad gamepad) {
        super(gamepad);
    }

    public void rumble (int durationMs){
        super.gamepad.rumble(durationMs);
    }

    public void rumble (double rumble1, double rumble2, int durationMs){
        super.gamepad.rumble(rumble1, rumble2, durationMs);
    }

    public float leftTrigger(){
        return super.gamepad.left_trigger;
    }

    public float rightTrigger(){
        return super.gamepad.right_trigger;
    }

    public Pose getGamepadInput(double driveScalar, double turnScalar){
        double left_stick_x = rootInput(super.gamepad.left_stick_x);
        double left_stick_y = rootInput(super.gamepad.left_stick_y);
        Vector2d drive = new Vector2d(left_stick_x, left_stick_y);

        double PositiveDriveAngle = Math.abs(drive.angle());
        double maximumNotScaledDownSpeedInARectangularContour = (PositiveDriveAngle >= Math.toRadians(45) && PositiveDriveAngle <= Math.toRadians(135)) ? Math.hypot(left_stick_x, 1) : Math.hypot(left_stick_y, 1);

        drive = drive.div(maximumNotScaledDownSpeedInARectangularContour);
        drive = drive.scale(drive.magnitude());

        double turn = -super.gamepad.right_stick_x;

        drive = drive.scale(driveScalar);
        turn *= turnScalar;
        return new Pose(drive, turn);
    }

    public Pose getGamepadInput(){
        return getGamepadInput(1, 1);
    }
    public Pose getGamepadInput(double driveScalar){
        return getGamepadInput(driveScalar, 1);
    }
    public float squareInput(double input) {return (float) (-Math.signum(input) * Math.pow(input, 2));}
    public float rootInput(double input) {return (float) (-Math.signum(input) * Math.sqrt(Math.abs(input)));}
}