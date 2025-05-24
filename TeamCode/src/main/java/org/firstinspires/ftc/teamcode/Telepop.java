package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp(name="Telepop")
public class Telepop extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    double Speed = 1;



    @Override public void runOpMode() {



        stuff.initializeTele();

        stuff.startlocation(0,0,0);

        stuff.telemetryupdate();

        stuff.DriveUpdateTele(0,0,0,0);


        waitForStart();



        while (opModeIsActive()) {


            stuff.telemetryupdate();

            stuff.DriveUpdateTele( -(gamepad1.left_stick_y), gamepad1.left_stick_x,gamepad1.right_stick_x, Speed);
            stuff.PersonalUpdateTele();

            if (0.1 <= gamepad1.left_trigger) {
                Speed = 0.25;
            } else if (0.1 <= gamepad1.right_trigger) {
                Speed = 1;
            } else {
                Speed = 0.5;
            }

            stuff.SlidesPosition(1);

            stuff.SlidesRetracted();

            stuff.CapstanPosition(1);

            if (gamepad1.a && gamepad1.y) {
                stuff.ResetPinpoint();
            }





        }
    }
}