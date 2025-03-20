package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Auto")
public class Aotu extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);






    public static int i;




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


            if (i == 1) {
                if (Timer.seconds() < 2) {
                    stuff.driveTo(0,0);
                    stuff.JohnBobDegree(60);
                    stuff.EmmaEmmrPosition(1000);
                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 2) {
                if (Timer.seconds() < 5) {
                    stuff.driveTo(20,0);
                    stuff.JohnBobDegree(0);
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 3) {
                if (Timer.seconds() < 2) {
                    stuff.JohnBobDegree(60);
                    stuff.EmmaEmmrPosition(1000);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 4) {
                if (Timer.seconds() < 5) {
                    stuff.driveTo(0,0);
                    stuff.JohnBobDegree(0);
                    stuff.EmmaEmmrPosition(0);
                } else {
                    i += 1;
                    Timer.reset();
                        }
            }else if (i == 5) {
                if (true) {
                    break;
                }
            }
        }
    }
}