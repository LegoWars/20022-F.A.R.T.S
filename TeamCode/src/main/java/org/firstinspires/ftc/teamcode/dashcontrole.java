package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="dash controle")
public class dashcontrole extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);


    public static double testx;
    public static double testy;
    public static double testheading;
    public static double testjohnbobdegree;
    public static double testemmaemmrposition;




    @Override public void runOpMode() {



        stuff.startlocation(0,0,180);
        stuff.initializeAuto();

        stuff.telemetryupdate();



        waitForStart();

        stuff.controllerUpdateAuto();
        stuff.telemetryupdate();

        while (opModeIsActive()) {

            stuff.controllerUpdateAuto();
            stuff.telemetryupdate();

            stuff.driveTo(testx, testy, testheading);
            stuff.JohnBobDegree(testjohnbobdegree);
            stuff.EmmaEmmrPosition(testemmaemmrposition);



        }
    }
}