package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="RED FAR loading limelight")
public class RedFarLoadingLimelight extends AutoPid {
    public RedFarLoadingLimelight() {
        customizable.stringBuilder = customizable.farLoadingLimelight;
    }
}
