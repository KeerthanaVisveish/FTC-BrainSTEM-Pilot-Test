package org.firstinspires.ftc.teamcode.opmode.autosBase;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="RED LIMELIGHT TESTING AUTO")
public class RedLimelightTestingAuto extends AutoPid {
    public RedLimelightTestingAuto() {
        alliance = Alliance.RED;
    }
}
