package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Config
@Autonomous(name="Auto testtest")
public class Aotutesttest extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);


    public static double testx;
    public static double testy;
    public static double testheading;
    public static double testdegree;
    public static int i;
    public static double emmaemmrpos;



    @Override public void runOpMode() {

        ElapsedTime Timer;

        Timer = new ElapsedTime();

        stuff.initializeAuto();

        stuff.telemetryupdate();



        waitForStart();
        Timer.reset();
        i = 1;

        while (opModeIsActive()) {
            stuff.controllerUpdateAuto();
            stuff.telemetryupdate();


//            stuff.driveTo(testx,testy,testheading);
            stuff.JohnBobDegree(testdegree);
            stuff.EmmaEmmrPosition(emmaemmrpos);

            if (i == 1) {
                if (Timer.seconds() < 4) {

                    stuff.JohnBobDegree(60);
                    stuff.EmmaEmmrPosition(1000);
                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 2) {
                if (Timer.seconds() < 10) {

                    stuff.JohnBobDegree(30);
                    stuff.EmmaEmmrPosition(00);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 3) {
                if (Timer.seconds() < 4) {
                    stuff.JohnBobDegree(60);
                    stuff.EmmaEmmrPosition(1000);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 4) {
                if (Timer.seconds() < 4) {
                    stuff.JohnBobDegree(30);
                    stuff.EmmaEmmrPosition(00);
                } else {
                    i += 1;
                    Timer.reset();
                        }
            }else if (i == 5) {
                if (Timer.seconds() < 10) {
                    stuff.JohnBobDegree(60);
                    stuff.EmmaEmmrPosition(1000);
                } else {
                    i += 1;
                    Timer.reset();
                }
            }

        }
    }
}