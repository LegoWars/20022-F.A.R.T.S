package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoClips")
public class AotuClips extends LinearOpMode {

    private Kitchen stuff = new Kitchen(this);
    public static int i = 1;


    @Override public void runOpMode() {

        ElapsedTime Timer;
        Timer = new ElapsedTime();

        stuff.startlocation(0,0,180);
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
                if (Timer.seconds() < 0.75) {

                    //drive to clip preload
                    stuff.driveTo(27,-3,180);
                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 2) {
                if (Timer.seconds() < 0.75) {

                    //setup to clip preload
                    stuff.JohnBobDegree(95);
                    stuff.NathanerPosition(0.8);
                    stuff.NathanestPosition(0.15);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 3) {
                if (Timer.seconds() < 0.25) {

                    //clip preload
                    stuff.EmmaEmmrPosition(650);

                } else {
                    i += 1;
                    Timer.reset();
                }
            }  else if (i == 4) {
                if (Timer.seconds() < 0.25) {

                    //release preload
                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 5) {
                if (Timer.seconds() < 0.25) {

                    //lower from preload
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 6) {
                if (Timer.seconds() < 1.25) {

                    //go around foot
                    stuff.JohnBobLow();
                    stuff.driveTo(15,-27);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 7) {
                if (Timer.seconds() < 0.5) {

                    //move next to f1
                    stuff.driveTo(50,-27);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 8) {
                if (Timer.seconds() < 0.25) {

                    //drive infront of f1
                    stuff.driveTo(50,-38,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 9) {
                if (Timer.seconds() < 1.5) {

                    //push f1
                    stuff.driveTo(5,-38);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 10) {
                if (Timer.seconds() < 1) {

                    //drive next to f2
                    stuff.driveTo(50,-38);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 11) {
                if (Timer.seconds() < 0.25) {

                    //drive infront of f2
                    stuff.driveTo(50,-48,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 12) {
                if (Timer.seconds() < 1) {

                    //push f2
                    stuff.driveTo(5,-46.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 13) {
                if (Timer.seconds() < 1) {

                    //move next to f3
                    stuff.driveTo(50,-45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 14) {
                if (Timer.seconds() < 0.25) {

                    //drive infront of f3
                    stuff.driveTo(50,-54.5,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 15) {
                if (Timer.seconds() < 1) {

                    //push f3
                    stuff.driveTo(7,-45);
                    stuff.JohnBobWall();
                    stuff.NathanerPosition(0.48);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 16) {
                if (Timer.seconds() < 0.5) {

                    //aline to get f1
                    stuff.driveTo(7,-38,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 17) {
                if (Timer.seconds() < 0.25) {

                    //drive to get f1
                    stuff.driveTo(1.5,-38,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 18) {
                if (Timer.seconds() < 0.25) {

                    //grab f1
                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 19) {
                if (Timer.seconds() < 0.25) {

                    //lift f1
                    stuff.JohnBobDegree(45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 20) {
                if (Timer.seconds() < 1.5) {

                    //drive aline to clip f1
                    stuff.driveTo(27,6,180);
                    stuff.JohnBobDegree(95);
                    stuff.NathanerPosition(0.8);
                    stuff.NathanestPosition(0.15);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 21) {
                if (Timer.seconds() < 0.25) {

                    //clip f1
                    stuff.EmmaEmmrPosition(650);

                } else {
                    i += 1;
                    Timer.reset();
                }
            }  else if (i == 22) {
                if (Timer.seconds() < 0.25) {

                    //release f1
                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 23) {
                if (Timer.seconds() < 0.25) {

                    //lower from f1
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 24) {
                if (Timer.seconds() < 1.75) {

                    //aline for f2
                    stuff.driveTo(8,-38,180);
                    stuff.JohnBobWall();
                    stuff.NathanerPosition(0.48);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 25) {
                if (Timer.seconds() < 0.25) {

                    //drive to grab f2
                    stuff.driveTo(1.5,-38,180,2.5);


                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 26) {
                if (Timer.seconds() < 0.25) {

                    //grab f2
                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 27) {
                if (Timer.seconds() < 0.25) {

                    //lift f2
                    stuff.JohnBobDegree(45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 28) {
                if (Timer.seconds() < 1.75) {

                    //drive aline to clip f2
                    stuff.driveTo(27,10,180);
                    stuff.JohnBobDegree(95);
                    stuff.NathanerPosition(0.8);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 29) {
                if (Timer.seconds() < 0.25) {

                    //clip f2
                    stuff.EmmaEmmrPosition(650);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 30) {
                if (Timer.seconds() < 0.25) {

                    //release f2
                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 31) {
                if (Timer.seconds() < 0.25) {

                    //lower from f2
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 32) {
                if (Timer.seconds() < 1.75) {

                    //aline for f3
                    stuff.driveTo(8,-38,180);
                    stuff.JohnBobWall();
                    stuff.NathanerPosition(0.48);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 33) {
                if (Timer.seconds() < 0.25) {

                    //drive to grab f3
                    stuff.driveTo(1.5,-38,180,2.5);


                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 34) {
                if (Timer.seconds() < 0.25) {

                    //grab f3
                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 35) {
                if (Timer.seconds() < 0.25) {

                    //lift f3
                    stuff.JohnBobDegree(45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 36) {
                if (Timer.seconds() < 1.75) {

                    //drive aline to clip f1
                    stuff.driveTo(27,14,180);
                    stuff.JohnBobDegree(95);
                    stuff.NathanerPosition(0.8);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 37) {
                if (Timer.seconds() < 0.25) {

                    //clip f3
                    stuff.EmmaEmmrPosition(650);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 38) {
                if (Timer.seconds() < 0.25) {

                    //release f3
                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 39) {
                if (Timer.seconds() < 0.25) {

                    //lower from f3
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 40) {
                if (Timer.seconds() < 1.75) {

                    //aline for pre 2
                    stuff.driveTo(8,-38,180);
                    stuff.JohnBobWall();
                    stuff.NathanerPosition(0.48);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 41) {
                if (Timer.seconds() < 0.25) {

                    //drive to grab pre 2
                    stuff.driveTo(1.5,-38,180,2.5);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 42) {
                if (Timer.seconds() < 0.25) {

                    //grab pre 2
                    stuff.NathanPosition(0.31);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 43) {
                if (Timer.seconds() < 0.25) {

                    //lift pre 2
                    stuff.JohnBobDegree(45);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 44) {
                if (Timer.seconds() < 1.75) {

                    //drive aline to clip pre 2
                    stuff.driveTo(27,18,180);
                    stuff.JohnBobDegree(95);
                    stuff.NathanerPosition(0.8);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 45) {
                if (Timer.seconds() < 0.25) {

                    //clip pre 2
                    stuff.EmmaEmmrPosition(650);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 46) {
                if (Timer.seconds() < 0.25) {

                    //release pre 2
                    stuff.NathanPosition(0.55);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 47) {
                if (Timer.seconds() < 0.25) {

                    //lower from pre 2
                    stuff.EmmaEmmrPosition(0);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 48) {
                if (Timer.seconds() < 1.5) {

                    //rest and park
                    stuff.JohnBobLow();
                    stuff.driveTo(4,-40);

                } else {
                    i += 1;
                    Timer.reset();
                }
            } else if (i == 49) {
                break;

            }
        }
    }
}