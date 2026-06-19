package org.firstinspires.ftc.teamcode.robot.shootingSystem.hood;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;

public abstract class Hood extends Component {

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

    public abstract void update();
    public abstract void setTargetExitAngle(double exitAngle);

    public abstract boolean onTarget();
}
