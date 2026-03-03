package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE FAR loading first")
public class BlueFarLoadingFirst extends BlueAutoPid {
    public BlueFarLoadingFirst() {
        customizable.stringBuilder = customizable.farLoadingFirst;
    }
}
