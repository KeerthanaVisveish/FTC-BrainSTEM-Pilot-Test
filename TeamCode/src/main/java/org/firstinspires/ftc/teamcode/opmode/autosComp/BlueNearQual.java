package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE NEAR partner")
public class BlueNearQual extends BlueAutoPid {
    public BlueNearQual() {
        customizable.stringBuilder = customizable.nearPartnerQual;
    }
}
