package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Aootoo")
public class Aootoo extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

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

            stuff.DriveUpdateAuto();
            stuff.PersonalUpdateTele();
            stuff.telemetryupdate();


            if (i == 1) {
                if (Timer.seconds() < 2) {
                    stuff.driveTo(0,0);
                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 2) {
                if (Timer.seconds() < 5) {
                    stuff.driveTo(20,0);
                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 3) {
                if (Timer.seconds() < 2) {
                    stuff.driveTo(10,0);
                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 4) {
                if (Timer.seconds() < 5) {
                    stuff.driveTo(0,0);

                } else {
                    i += 1;
                    Timer.reset();
                        }
            }else if (i == 5) {
                    break;
            }
        }
    }
}