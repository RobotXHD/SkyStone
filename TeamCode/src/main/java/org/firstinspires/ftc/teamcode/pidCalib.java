package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class pidCalib extends OpMode {
    DcMotorEx motordf, motords, motorsf, motorss;
    double p, i, d;
    @Override
    public void init() {
        motordf = hardwareMap.get(DcMotorEx.class, "motordf");
        motords = hardwareMap.get(DcMotorEx.class, "motords");
        motorsf = hardwareMap.get(DcMotorEx.class, "motorsf");
        motorss = hardwareMap.get(DcMotorEx.class, "motorss");
    }

    @Override
    public void loop() {

    }
}
