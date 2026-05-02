package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE NEAR playoff")
public class BlueNearPlayoff extends BlueAutoPid {
    public BlueNearPlayoff() {
        customizable.stringBuilder = customizable.nearPartnerPlayoff;
    }
}
