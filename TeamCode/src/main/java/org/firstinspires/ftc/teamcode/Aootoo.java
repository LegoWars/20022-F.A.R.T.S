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
    ElapsedTime Timer;

    public void Counter(double Time){
        if (Timer.seconds() > Time) {
            i += 1;
            Timer.reset();
        }
    }



    @Override public void runOpMode() {


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
                    stuff.driveTo(0,0);
            Counter(2);
            }

            else if (i == 2) {
                    stuff.driveTo(20,0);
            Counter(5);
            }

            else if (i == 3) {
                    stuff.driveTo(10,0);
            Counter(2);
            }

            else if (i == 4) {
                    stuff.driveTo(0,0);
            Counter(5);
            }

            else if (i == 5) {
                    break;
            }

        }
    }
}