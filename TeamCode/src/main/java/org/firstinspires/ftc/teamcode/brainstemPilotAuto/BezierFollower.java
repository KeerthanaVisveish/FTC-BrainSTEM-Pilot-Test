package org.firstinspires.ftc.teamcode.brainstemPilotAuto;

import com.acmerobotics.dashboard.config.Config;

import org.brainstemfirst.pilot.ftc.bezier.follower.BezierFollowerConfig;

/**
 * FTC Dashboard knobs for the Bézier follower. Copied onto {@link BezierFollowerConfig}
 * every loop so live edits apply immediately. Dashboard only scans TeamCode, not the library.
 */
@Config
public class BezierFollower {
    public static boolean useVelocityProfile = true;
    public static double velKv = 0.014;
    public static double velKs = 0.03;
    public static double velKp = 0.05;

    public static boolean overrideCruiseVel = false;
    public static double cruiseVel = 30;
    public static boolean overrideProfileDecel = false;
    public static double profileDecel = 40;

    public static double crossTrackKp = 0.12;
    public static double correctivePower = 0.7;

    public static double headingkP = 0.35;
    public static double headingkD = 0.02;
    public static double headingkF = 0.0;
    public static double headingFfDeadbandDeg = 8.0;

    public static double speedkP = 0.05;
    public static double speedkF = 0.05;
    public static double speedkD = 0.0;

    public static void apply() {
        BezierFollowerConfig.useVelocityProfile = useVelocityProfile;
        BezierFollowerConfig.velKv = velKv;
        BezierFollowerConfig.velKs = velKs;
        BezierFollowerConfig.velKp = velKp;
        BezierFollowerConfig.overrideCruiseVel = overrideCruiseVel;
        BezierFollowerConfig.cruiseVel = cruiseVel;
        BezierFollowerConfig.overrideProfileDecel = overrideProfileDecel;
        BezierFollowerConfig.profileDecel = profileDecel;
        BezierFollowerConfig.crossTrackKp = crossTrackKp;
        BezierFollowerConfig.correctivePower = correctivePower;
        BezierFollowerConfig.headingkP = headingkP;
        BezierFollowerConfig.headingkD = headingkD;
        BezierFollowerConfig.headingkF = headingkF;
        BezierFollowerConfig.headingFfDeadbandDeg = headingFfDeadbandDeg;
        BezierFollowerConfig.speedkP = speedkP;
        BezierFollowerConfig.speedkF = speedkF;
        BezierFollowerConfig.speedkD = speedkD;
    }
}
