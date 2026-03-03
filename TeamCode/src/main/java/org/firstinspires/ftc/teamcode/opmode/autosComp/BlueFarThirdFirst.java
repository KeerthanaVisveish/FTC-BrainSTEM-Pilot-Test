package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE FAR third first")
public class BlueFarThirdFirst extends BlueAutoPid {
    public BlueFarThirdFirst() {
        customizable.stringBuilder = customizable.thirdFirstFar;
    }
}
