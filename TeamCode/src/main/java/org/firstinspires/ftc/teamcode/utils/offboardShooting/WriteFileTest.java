package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name="WriteFileTest")
public class WriteFileTest extends OpMode {
    @Override
    public void init() {
        File file = AppUtil.getInstance().getSettingsFile("mtiTrajectories.json");
        ReadWriteFile.writeFile(file, MtiJsonStringGetter.getJson());
        telemetry.addLine("File Written");
        telemetry.addData("contents", MtiJsonStringGetter.getJson().substring(0, 500));
    }

    @Override
    public void loop() {

    }
}
