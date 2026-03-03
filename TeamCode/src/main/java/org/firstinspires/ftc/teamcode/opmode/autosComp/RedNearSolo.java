package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED NEAR solo")
public class RedNearSolo extends RedAutoPid {
    public RedNearSolo() {
        customizable.stringBuilder = customizable.soloNear;
    }
}
