package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE custom")
public class BlueCustom extends BlueAutoPid {
    public BlueCustom() {
        customizable.stringBuilder = customizable.custom;
    }
}
