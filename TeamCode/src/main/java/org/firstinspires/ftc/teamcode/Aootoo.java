package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="Aootoo")
public class Aootoo extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i;


    @Override public void runOpMode() {


        stuff.initializeAuto();
        stuff.telemetryupdate();


        waitForStart();

        stuff.StartUpAuto();


        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.PersonalUpdateTele();
            stuff.telemetryupdate();


            if (i == 1) {
                    stuff.driveTo(0,0);
            stuff.Counter(2);
            }

            else if (i == 2) {
                    stuff.driveTo(20,0);
            stuff.Counter(5);
            }

            else if (i == 3) {
                    stuff.driveTo(10,0);
            stuff.Counter(2);
            }

            else if (i == 4) {
                    stuff.driveTo(0,0);
            stuff.Counter(5);
            }

            else if (i == 5) {
                    break;
            }

        }
    }
}