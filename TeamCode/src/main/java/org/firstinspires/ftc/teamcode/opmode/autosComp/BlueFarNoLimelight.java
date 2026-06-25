package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE FAR no limelight")
public class BlueFarNoLimelight extends BlueAutoPid {
    public BlueFarNoLimelight() {
        customizable.stringBuilder = customizable.farNoLimelight;
    }
}
