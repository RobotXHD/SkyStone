package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

@TeleOp
public class TeleOP_S4T_LOCC extends OpMode {


    private DcMotor encoderDreapta , encoderSpate, encoderStanga;
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private int v = 2;
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    private double forward, rright, clockwise;
    private boolean stop = false;
    private long fpsC=0, fpsEncDr = 0, fpsEncSp = 0, fpsEncSt = 0, fpsLOC = 0, fpsLOCALIZARE = 0;
    private long sysTimeC, sysTimeEncSp, sysTimeEncDr, sysTimeEncSt, sysTimeLOC, sysTimeLOCALIZARE,sysTimeLoop, sysTimeP, sysTimeI, sysTime, nextLoopTime = 3000, nextLoop;
    private long fpsCLast, fpsEncSpLast, fpsEncDrLast, fpsEncStLast, fpsLOCLast, fpsLOCALIZARELast;
    private int EncDr, EncSp, EncSt;
    private double angle, fs, sideways;
    private double d = 377, omniLenght = 188.49555921538759430775860299677, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLenght / 360) * 4000;
    private double encRot, targetAngle, delta, p, i, kp = -0.16, ki = -0.3, kd, corr;
    private boolean rot = true, EncDrVerificare = false, EncStVerificare = false, EncSpVerificare = false;


    private Thread ENCDr = new Thread(new Runnable() {
        int encdr;
        @Override
        public void run() {
            while(!stop){
                if(!EncDrVerificare) {
                    EncDrVerificare = true;
                    encdr = encoderDreapta.getCurrentPosition();
                    EncDr = encdr;
                }
                fpsEncDr++;
                if (sysTimeEncDr + 3000 < System.currentTimeMillis()) {
                        fpsEncDrLast = fpsEncDr / 3;
                        fpsEncDr = 0;
                        sysTimeEncDr = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCSp = new Thread(new Runnable() {
        int encsp;
        @Override
        public void run() {
            while(!stop){
                if(!EncSpVerificare) {
                    EncSpVerificare = true;
                    encsp = encoderSpate.getCurrentPosition();
                    EncSp = encsp;
                }
                fpsEncSp++;
                if (sysTimeEncSp + 3000 < System.currentTimeMillis()) {
                        fpsEncSpLast = fpsEncSp / 3;
                        fpsEncSp = 0;
                        sysTimeEncSp = System.currentTimeMillis();
                }

            }

        }
    });

    private Thread ENCSt = new Thread(new Runnable() {
        int encst;
        @Override
        public void run() {
            while(!stop) {
                if (!EncStVerificare) {
                    EncStVerificare = true;
                    encst = encoderStanga.getCurrentPosition();
                    EncSt = -encst;
                }
                fpsEncSt++;
                if (sysTimeEncSt + 3000 < System.currentTimeMillis()) {
                    fpsEncStLast = fpsEncSt / 3;
                    fpsEncSt = 0;
                    sysTimeEncSt = System.currentTimeMillis();
                }

            }
        }
    });


    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }
                forward = gamepad1.left_stick_y;
                rright = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                df = forward + clockwise - rright;
                ss = forward - clockwise - rright;
                sf = -forward + clockwise - rright;
                ds = -forward - clockwise - rright;

                max = abs(sf);
                if (abs(df) > max) {
                    max = abs(df);
                }
                if (abs(ss) > max) {
                    max = abs(ss);
                }
                if (abs(ds) > max) {
                    max = abs(ds);

                }
                if (max > 1) {
                    sf /= max;
                    df /= max;
                    ss /= max;
                    ds /= max;
                }

                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }

                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();

                }
            }
        }
    });

    private Thread LOC = new Thread(new Runnable() {
        double offset = 0;
        @Override
        public void run() {
            while (!stop) {
                if(gamepad1.y){
                    offset = (EncDr - EncSt) / 2.0;
                }
                encRot = ((EncDr - EncSt) / 2.0) - offset;
                angle = encRot / tickPerDeg;

                fpsLOC++;
                if (sysTimeLOC + 3000 < System.currentTimeMillis()) {
                    fpsLOCLast = fpsLOC/3;
                    fpsLOC = 0;
                    sysTimeLOC = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread LOCALIZARE = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                Math.sin(angle);

                fpsLOCALIZARE++;
                if (sysTimeLOCALIZARE + 3000 < System.currentTimeMillis()) {
                    fpsLOCALIZARELast = fpsLOCALIZARE/3;
                    fpsLOCALIZARE = 0;
                    sysTimeLOCALIZARE = System.currentTimeMillis();
                }
            }

        }
    });


    @Override
    public void init() {

        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        encoderDreapta = hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga = hardwareMap.get(DcMotor.class, "encoderStanga");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sysTime = System.currentTimeMillis();
        sysTimeC = sysTime;
        sysTimeEncDr = sysTime;
        sysTimeEncSp = sysTime;
        sysTimeEncSt = sysTime;
        sysTimeLOC = sysTime;
        sysTimeP = sysTime;
        sysTimeI = sysTime;
        sysTimeLOCALIZARE = sysTime;

        ENCDr.start();
        ENCSp.start();
        ENCSt.start();
        Chassis.start();
        LOC.start();
        LOCALIZARE.start();
    }

    @Override
    public void loop() {
        telemetry.addData("ENCDr:", EncDr);
        telemetry.addData("ENCSp:", EncSp);
        telemetry.addData("ENCSt:", EncSt);
        telemetry.addData("EncRot", encRot);

        telemetry.addData("Angle:", angle);

        telemetry.addData("motordf: ", df);
        telemetry.addData("motorsf: ", sf);
        telemetry.addData("motords: ", ds);
        telemetry.addData("motorss: ", ss);

        telemetry.addData("p: ", p);
        telemetry.addData("i: ", i);
        telemetry.addData("kp: ", kp);
        telemetry.addData("ki: ", ki);

        /*telemetry.addData("Th Chassis: ", fpsCLast);
        telemetry.addData("Th EncSp: ", fpsEncSpLast);
        telemetry.addData("Th EncDr: ", fpsEncDrLast);
        telemetry.addData("Th EncSt: ", fpsEncStLast);
        telemetry.addData("Th LOC: ", fpsLOCLast);*/
        telemetry.addData("Th LOCALIZARE: ", fpsLOCALIZARELast);
        telemetry.update();
    }

    public void stop(){stop = true;}


    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}