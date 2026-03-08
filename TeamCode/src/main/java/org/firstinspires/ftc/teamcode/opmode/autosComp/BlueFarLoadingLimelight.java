package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE FAR loading limelight")
public class BlueFarLoadingLimelight extends BlueAutoPid {
    public BlueFarLoadingLimelight() {
        customizable.stringBuilder = customizable.farLoadingLimelight;
    }
}
