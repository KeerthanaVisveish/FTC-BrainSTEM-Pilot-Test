package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;

@TeleOp(name="ReadFileTest")
public class ReadFileTest extends OpMode {
    @Override
    public void init() {
        File file = AppUtil.getInstance().getSettingsFile("mtiTrajectories.json");
        String contents = ReadWriteFile.readFile(file);
        telemetry.addData("contents", contents.substring(0, 400));
        telemetry.update();

        TrajectoryDistanceLUT lut = TrajectoryLoader.loadFromSettingsFile("mtiTrajectories.json");
        telemetry.addData("1m in range", lut.distanceInRange(1));
        telemetry.addData("3m in range", lut.distanceInRange(3));
        telemetry.addData("5m in range", lut.distanceInRange(5));
        telemetry.addData("6m in range", lut.distanceInRange(6));

        telemetry.addData("optimal traj", lut.getInterpolatedOptimalTrajectory(5));
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
