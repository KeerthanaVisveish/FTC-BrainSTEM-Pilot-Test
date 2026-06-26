package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED FAR loading first")
public class RedFarLoadingFirst extends RedAutoPid {
    public RedFarLoadingFirst() {
        customizable.stringBuilder = customizable.farLoadingFirst;
        customizable.gatePark = false;
    }
}
