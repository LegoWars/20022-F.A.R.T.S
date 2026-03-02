package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="dash controle")
public class dashcontrole extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);


    public static double testx;
    public static double testy;
    public static double testheading;




    @Override public void runOpMode() {



        stuff.startlocation(0,0,0);
        stuff.initializeAuto();

        stuff.telemetryupdate();



        waitForStart();

        stuff.StartUpAuto();
        stuff.telemetryupdate();

        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.telemetryupdate();

            stuff.driveTo(testx, testy, testheading);




        }
    }
}