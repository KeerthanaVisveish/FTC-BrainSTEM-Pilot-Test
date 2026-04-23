package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE NEAR pattern")
public class BlueNearPattern extends BlueAutoPid {
    public BlueNearPattern() {
        customizable.shouldColorSort = true;
    }
}
