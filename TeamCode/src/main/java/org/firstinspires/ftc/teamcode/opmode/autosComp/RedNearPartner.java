package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED NEAR partner")
public class RedNearPartner extends RedAutoPid {
    public RedNearPartner() {
        customizable.stringBuilder = customizable.partnerNear;
    }
}
