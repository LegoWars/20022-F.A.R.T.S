package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Tuning")
public class Tuning extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);


    public static double tunex;
    public static double tuney;
    public static double tuneheading;
    public static double bannanadegree;



    @Override public void runOpMode() {

        ElapsedTime Timer;

        Timer = new ElapsedTime();
        stuff.initializeAuto();

        stuff.telemetryupdate();





        waitForStart();
        Timer.reset();
        tunex = 0;
        tuney = 0;
        tuneheading = 0;


        while (opModeIsActive()) {
            stuff.controllerUpdateAuto();
            stuff.telemetryupdate();



//                if (Timer.seconds() < 4) {
//
//                    stuff.driveTo(0, 0,0);
//
//                } else if (Timer.seconds() < 8) {
//
//                    stuff.driveTo(0, 0, 0);
//
//                } else {
//                    Timer.reset();
//                }



        }
    }
}