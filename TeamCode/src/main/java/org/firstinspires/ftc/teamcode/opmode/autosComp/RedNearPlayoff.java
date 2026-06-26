package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED NEAR playoff")
public class RedNearPlayoff extends RedAutoPid {
    public RedNearPlayoff() {
        customizable.stringBuilder = customizable.nearPartnerPlayoff;
        customizable.gatePark = true;
    }
}
