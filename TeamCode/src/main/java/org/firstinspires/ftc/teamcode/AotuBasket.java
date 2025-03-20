package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoBasket")
public class AotuBasket extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);
    public static int i = 1;


    @Override public void runOpMode() {

        ElapsedTime Timer;
        Timer = new ElapsedTime();

        stuff.startlocation(0,0,90);
        stuff.initializeAuto();
        stuff.telemetryupdate();



        waitForStart();

        i = 1;
        Timer.reset();

        while (opModeIsActive()) {

            stuff.controllerUpdateAuto();
            stuff.telemetryupdate();
            telemetry.addData("i", i);


            if (i == 1) {
                if (Timer.seconds() < 0.5) {

                    //move off wall
                    stuff.driveTo(7,0,90);
                    stuff.NathanPosition(0.31);
                    stuff.NathanerPosition(0.58);
                    stuff.NathanestPosition(0.15);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 2) {
                if (Timer.seconds() < 1.5) {

                    //drive over to drop off preload
                    stuff.driveTo(7,19,45);
                    stuff.JohnBobHighBasket();
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 3) {
                if (Timer.seconds() < 0.25) {

                    //extend for preload
                    stuff.EmmaEmmrPosition(1400);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 4) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanerPosition(0.75);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 5) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 6) {
                if (Timer.seconds() < 0.5) {

                    //retracked preload
                    stuff.NathanerPosition(0.58);
                    stuff.driveTo(9,17,45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 7) {
                if (Timer.seconds() < 0.25) {

                    //retracked preload
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 8) {
                if (Timer.seconds() < 1.75) {

                    //drive to pickup floor 1
                    stuff.driveTo(15.5,10,0);
                    stuff.JohnBobFloor();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 9) {
                if (Timer.seconds() < 0.5) {

                    //extend for floor 1
                    stuff.EmmaEmmrPosition(800);
                    stuff.NathanPosition(0.50);
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 10) {
                if (Timer.seconds() < 0.25) {

                    //lower toopick up F1
                    stuff.JohnBobLow();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 11) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 12) {
                if (Timer.seconds() < 0.25) {

                    //retracte floor 1
                    stuff.EmmaEmmrPosition(0);
                    stuff.NathanerPosition(0.58);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 13) {
                if (Timer.seconds() < 1.5) {

                    //drive to drop off floor 1
                    stuff.driveTo(7,19,45);
                    stuff.JohnBobHighBasket();
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 14) {
                if (Timer.seconds() < 0.25) {

                    //extend dropoff F1
                    stuff.EmmaEmmrPosition(1400);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 15) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanerPosition(0.75);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 16) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 17) {
                if (Timer.seconds() < 0.5) {

                    //retracked dropoff F1
                    stuff.NathanerPosition(0.58);
                    stuff.driveTo(9,17,45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 18) {
                if (Timer.seconds() < 0.25) {

                    //retracked dropoff F1
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 19) {
                if (Timer.seconds() < 1.75) {

                    //drive to pick up floor 2
                    stuff.driveTo(15.5,20,0);
                    stuff.JohnBobFloor();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 20) {
                if (Timer.seconds() < 0.5) {

                    //extend for floor 2
                    stuff.EmmaEmmrPosition(800);
                    stuff.NathanPosition(0.50);
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 21) {
                if (Timer.seconds() < 0.25) {

                    //lower toopick up F2
                    stuff.JohnBobLow();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 22) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 23) {
                if (Timer.seconds() < 0.25) {

                    //retracket floor 2
                    stuff.EmmaEmmrPosition(0);
                    stuff.NathanerPosition(0.58);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 24) {
                if (Timer.seconds() < 1.5) {

                    //drive to dropoff floor 2
                    stuff.driveTo(7,19,45);
                    stuff.JohnBobHighBasket();
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 25) {
                if (Timer.seconds() < 0.25) {

                    //extend dropoff F2
                    stuff.EmmaEmmrPosition(1400);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 26) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanerPosition(0.75);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 27) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 28) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanerPosition(0.58);
                    //retracked dropoff F2
                    stuff.driveTo(9,17,45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 29) {
                if (Timer.seconds() < 0.25) {

                    //retracked dropoff F2
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 30) {
                if (Timer.seconds() < 2.5) {

                    //drive to pickup floor 3
                    stuff.driveTo(37,7,270);
                    stuff.JohnBobFloor();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 31) {
                if (Timer.seconds() < 0.5) {

                    //extend to pick floor 3
                    stuff.EmmaEmmrPosition(800);
                    stuff.NathanPosition(0.50);
                    stuff.NathanerPosition(0.18);
                    stuff.NathanestPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 32) {
                if (Timer.seconds() < 0.25) {

                    //lower toopick up F3
                    stuff.JohnBobLow();

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 33) {
                if (Timer.seconds() < 0.25) {

                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 34) {
                if (Timer.seconds() < 0.25) {

                    //retracket pick up floor 3
                    stuff.EmmaEmmrPosition(0);
                    stuff.NathanerPosition(0.58);
                    stuff.NathanestPosition(0.15);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 35) {
                if (Timer.seconds() < 2) {

                    //drive to dropoff floor 3
                    stuff.driveTo(7,19,45);
                    stuff.JohnBobHighBasket();
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 36) {
                if (Timer.seconds() < 0.25) {

                    //extend dropoff F3
                    stuff.EmmaEmmrPosition(1400);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 37) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanerPosition(0.75);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 38) {
                if (Timer.seconds() < 0.5) {

                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 39) {
                if (Timer.seconds() < 0.5) {

                    //retracked dropoff F3
                    stuff.driveTo(9,17,45);
                    stuff.NathanerPosition(0.58);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 40) {
                if (Timer.seconds() < 0.25) {

                    //retracked dropoff F3
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 41) {
                if (Timer.seconds() < 1.5) {

                    //drive to park
                    stuff.driveTo(55,0,90);
                    stuff.NathanerPosition(0.18);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 42) {
                if (Timer.seconds() < 1.5) {

                    //lower for reset
                    stuff.driveTo(55,-18.75,180);


                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 43) {
                break;

            }
        }
    }
}