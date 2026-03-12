package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED custom")
public class RedCustom extends RedAutoPid {
    public RedCustom() {
        customizable.stringBuilder = customizable.custom;
    }
}
