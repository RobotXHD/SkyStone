package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.channels.DatagramChannel;

public class HardwareSkybot extends LinearOpMode {
    public DcMotorEx motorss, motorsf, motords, motordf;
    public DcMotor encoderdr,encodersp,encoderst;
    public HardwareMap hwmap;

    public HardwareSkybot(){

    }


    public void init(HardwareMap hmap){
        hwmap = hmap;
        motorss = hwmap.get(DcMotorEx.class, "ss");
        motords = hwmap.get(DcMotorEx.class, "ds");
        motorsf = hwmap.get(DcMotorEx.class, "sf");
        motordf = hwmap.get(DcMotorEx.class, "df ");

        encoderdr = hwmap.get(DcMotor.class, "encoderDreapta");
        encodersp = hwmap.get(DcMotor.class, "encoderSpate");
        encoderst = hwmap.get(DcMotor.class, "encoderStanga");

        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motords.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorsf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorss.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    @Override
    public void runOpMode(){

    }
}
