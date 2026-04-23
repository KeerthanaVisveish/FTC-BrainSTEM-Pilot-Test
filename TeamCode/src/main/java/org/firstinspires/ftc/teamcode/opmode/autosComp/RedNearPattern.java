package org.firstinspires.ftc.teamcode.opmode.autosComp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autosBase.RedAutoPid;

@Autonomous(name="RED NEAR pattern")
public class RedNearPattern extends RedAutoPid {
    public RedNearPattern() {
        customizable.shouldColorSort = true;
    }
}
