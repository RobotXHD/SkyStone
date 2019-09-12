package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

@TeleOp
public class TeleOP_S4T_RTH extends OpMode {

    private DcMotor encoderDreapta , encoderSpate;
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
    private long fpsC=0 , fpsEncD= 0 , fpsEncS = 0 ;
    private long sysTimeC , sysTimeEncS , sysTimeEncD, sysTimeLoop, nextLoopTime = 3000, nextLoop;
    private long fpsCLast , fpsEncSLast , fpsEncDLast;
    private long targetFS, targetSD;
    private long EncDr , EncSp;


    private Thread ENCD = new Thread(new Runnable() {
        long encdr;
        @Override
        public void run() {
            while(!stop){
                encdr = encoderDreapta.getCurrentPosition();
                EncDr = encdr;

                fpsEncD++;
                if (sysTimeEncD + 3000 < System.currentTimeMillis()) {
                    fpsEncDLast = fpsEncD / 3;
                    fpsEncD = 0;
                    sysTimeEncD= System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCS = new Thread(new Runnable() {
        long encsp;
        @Override
        public void run() {
            while(!stop){
                encsp =  encoderSpate.getCurrentPosition();
                EncSp =  encsp;

                fpsEncS++;
                if (sysTimeEncS + 3000 < System.currentTimeMillis()) {
                    fpsEncSLast = fpsEncS/3;
                    fpsEncS = 0;
                    sysTimeEncS = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if(gamepad1.a){
                    targetFS = EncDr;
                    targetSD = EncSp;
                }
                else if(gamepad1.b){
                    if(targetFS < EncDr){
                        while(targetFS < EncDr-100 && !stop){
                            POWER(0.1, -0.1, -0.1, 0.1);
                        }
                    }
                    else if(targetFS > EncDr){
                        while(targetFS > EncDr+100 && !stop){
                            POWER(-0.1, 0.1, 0.1, -0.1);
                        }
                    }

                    if(targetSD < EncSp){
                        while (targetSD < EncSp - 50 && !stop){
                            POWER(0.1, 0.1, 0.1, 0.1);
                        }
                    }
                    else if(targetSD > EncSp){
                        while (targetSD > EncSp + 50 && !stop){
                            POWER(-0.1, -0.1, -0.1, -0.1);
                        }
                    }
                }
                else {
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
                    fpsC++;
                    if (sysTimeC + 1000 < System.currentTimeMillis()) {
                        fpsCLast = fpsC;
                        fpsC = 0;
                        sysTimeC = System.currentTimeMillis();
                    }

                    if (v == 1) {
                        POWER(df / 5, sf / 5, ds / 5, ss / 5);
                    } else if (v == 2) {
                        POWER(df, sf, ds, ss);
                    }

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

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sysTimeC = System.currentTimeMillis();
        sysTimeEncD = System.currentTimeMillis();
        sysTimeEncS = System.currentTimeMillis();

        ENCD.start();
        ENCS.start();
        Chassis.start();

    }

    @Override
    public void loop(){
            telemetry.addData("ENCDR:", EncDr);
            telemetry.addData("ENCSP:", EncSp);
            telemetry.addData("motordf: ", motordf.getCurrentPosition());
            telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
            telemetry.addData("motords: ", motords.getCurrentPosition());
            telemetry.addData("motorss: ", motorss.getCurrentPosition());
            telemetry.addData("Th Chassis: ", fpsCLast);
            telemetry.addData("Th EncS: ", fpsEncSLast);
            telemetry.addData("Th EncD: ", fpsEncDLast);
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
