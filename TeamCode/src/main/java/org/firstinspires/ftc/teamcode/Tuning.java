package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Tuning")
public class Tuning extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);


    public static double tunex;
    public static double tuney;
    public static double tuneheading;



    @Override public void runOpMode() {

        ElapsedTime Timer;

        Timer = new ElapsedTime();


        stuff.initializeAuto();

        stuff.telemetryupdate();


        waitForStart();

        stuff.StartUpAuto();

        Timer.reset();
        tunex = 0;
        tuney = 0;
        tuneheading = 0;


        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.telemetryupdate();



                if (Timer.seconds() < 5) {

                    stuff.driveTo(0, 0,0);

                } else if (Timer.seconds() < 10) {

                    stuff.driveTo(tunex, tuney, tuneheading);

                } else {
                    Timer.reset();
                }



        }
    }
}