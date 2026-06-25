package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.BlueAutoPid;

@Autonomous(name="BLUE FAR 18")
public class BlueFar18 extends BlueAutoPid {
    public BlueFar18() {
        customizable.stringBuilder = customizable.far18;
    }
}
