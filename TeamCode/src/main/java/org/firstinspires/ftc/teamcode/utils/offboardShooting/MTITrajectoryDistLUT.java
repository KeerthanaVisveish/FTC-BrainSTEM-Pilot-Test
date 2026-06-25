package org.firstinspires.ftc.teamcode.utils.offboardShooting;

public class MTITrajectoryDistLUT {
    public static TrajectoryDistanceLUT get() {
        return TrajectoryLoader.loadFromSettingsFile("mtitrajectories.json");
    }
}
