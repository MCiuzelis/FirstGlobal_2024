package org.firstinspires.ftc.teamcode.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.utils.wrappers.BetterGamepad;

public class rumbleCommand extends InstantCommand {
    public rumbleCommand(BetterGamepad gamepad, int duration){
        super(()->gamepad.rumble(duration));
    }
}
