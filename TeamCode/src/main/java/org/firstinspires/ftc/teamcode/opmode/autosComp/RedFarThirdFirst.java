package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED FAR third first")
public class RedFarThirdFirst extends RedAutoPid {
    public RedFarThirdFirst() {
        customizable.stringBuilder = customizable.farThirdFirst;
    }
}
