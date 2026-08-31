
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.geometry.Vector2d;



@Autonomous(name = "CloseRed6Odo this is the weird 1")
public class Aoo2 extends LinearOpMode {

    //inst
    DcMotor FL, FR, BL, BR;
    DcMotor Intake, Launcher;
    CRServo Loader;
    GoBildaPinpointDriver pinp;
    double lastError = 0;
    //tune
    static final double KD_DRIVE = 0.0;
    static final double KP_DRIVE = 0.025;
    static final double KP_TURN  = 0.2;
    static final double MAX_DRIVE_POWER = 1.0;
    static final double MIN_DRIVE_POWER = 0.15;


    // Inputs
    double YourCurrentx = 0;
    double YourCurrenty = 0;
    double YourCurrentheading = 0;
    double YourTargetx = 0;
    double YourTargety = 0;
    double YourTargetheading = 0;



    //Outputs
    double FrontRightPower = 0;
    double FrontLeftPower = 0;
    double BackRightPower = 0;
    double BackLeftPower = 0;



    //defining variables
    double DrivePower = 0;
    double StrafePower = 0;
    double HeadingPower = 0;


    double xTarget = 0;
    double yTarget = 0;
    double headingTarget = 0;
    double xCurrent = 0;
    double yCurrent = 0;
    double headingCurrent = 0;


    double LastErrorDrive = 0;
    double IntThingdrive = 0;
    double derThingDrive = 0;
    double DriveError = 0;


    double LastStrafeError = 0;
    double IntThingStrafe = 0;
    double derThingStrafe = 0;
    double StrafeError = 0;


    double LastErrorHeading = 0;
    double intThingheading = 0;
    double derThingHeading = 0;
    double HeadingError = 0;



    //setting constants
    double pDrive = 0.035;
    double iDrive = 0;
    double dDrive = 0;
    //all must be tuned for smooth operation


    double pStrafe = 0.035;
    double iStrafe = 0;
    double dStrafe = 0;
    //all must be tuned for smooth operation


    double pHeading = 0.01;
    double iHeading = 0;
    double dHeading = 0;
    //all must be tuned for smooth operation


    double Speed = 1;
    //this can be used so parts of you robot operation is slow and accurate and others are fast but not so accurate


    Vector2d errorVector;
    Vector2d TargetVector;


    ElapsedTime timerthing = new ElapsedTime();


    @Override
    public void runOpMode() {


        //motor init
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Launcher = hardwareMap.get(DcMotor.class, "launcher");
        Loader = hardwareMap.get(CRServo.class, "loader");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Loader.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pinpoint init
        pinp = hardwareMap.get(GoBildaPinpointDriver.class, "pinp");
        pinp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ((DcMotorEx) Launcher).setVelocityPIDFCoefficients(25,5,5,5);
        pinp.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        telemetry.addLine("Pinpoint Ready");

        pinp.resetPosAndIMU();
        pinp.setPosX(0,DistanceUnit.INCH);
        pinp.setPosY(0,DistanceUnit.INCH);
        pinp.setOffsets(-3,-4,DistanceUnit.INCH);
        pinp.update();
        telemetry.addData("posxA", pinp.getPosX(DistanceUnit.INCH));
        telemetry.addData("posyA", pinp.getPosY(DistanceUnit.INCH));
        telemetry.addData("posheading", pinp.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        sleep(100);
        pinp.recalibrateIMU();
        sleep(100);
        waitForStart();

        if (isStopRequested()) return;

        //movements


        //Target = 1100;
        //((DcMotorEx) Launcher).setVelocity(Target);
        drive(12, 5,0,1,0);
        drive(0, 5,0,1,180);
        drive(0, 5,12,1,180);
        sleep(20000);
        //turnToHeading(180);
        //sleep(2000);
        //driveStraightInch(-12.0, 0);
        //sleep(2000);
        //driveStraightInch(24.0, 0);
        //driveSideways(10.0,0);
        //driveSideways(-10.0,0);
        //Loader.setPower(1.0);
        //Intake.setPower(1.0);
        //sleep(5000);
        //Loader.setPower(0);
        //turnToHeading(143);
        //driveSideways(-20, 135);
        //Intake.setPower(1.0);
        //driveStraightInch(-5, 135);
        //driveStraightInch(10, 135);
        //driveSideways(20, 135);
        //turnToHeading(0);

        //stopDrive();
    }


    void drive(double inches, double runtime, double strafe, double sped, double headding) {

        YourTargetx = pinp.getPosX(DistanceUnit.INCH) + inches;
        YourTargety = (-pinp.getPosY(DistanceUnit.INCH)) + strafe;
        headingTarget = headding;
        timerthing.reset();


        while (timerthing.seconds() < runtime && opModeIsActive()) {

            pinp.update();

            Pose2D CurrentLocation = pinp.getPosition();
            xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
            yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
            headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);

            if (headingCurrent < 0) {
                headingCurrent = headingCurrent + 360;
            }
            errorVector = new Vector2d(YourTargetx-xCurrent,YourTargety-yCurrent);
            TargetVector = errorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));
            xTarget = TargetVector.getX();
            yTarget = TargetVector.getY();


            Speed = sped;



            DriveError = xTarget;
            IntThingdrive += DriveError;
            derThingDrive = DriveError - LastErrorDrive;
            DrivePower = (DriveError * pDrive) + (IntThingdrive * iDrive) + (derThingDrive * dDrive);
            LastErrorDrive = DriveError;



            StrafeError = yTarget;
            IntThingStrafe += StrafeError;
            derThingStrafe = StrafeError - LastStrafeError;
            StrafePower = (StrafeError * pStrafe) + (IntThingStrafe * iStrafe) + (derThingStrafe * dStrafe);
            LastStrafeError = StrafeError;



            HeadingError = headingTarget - headingCurrent;

            if (HeadingError > 180.0){
                HeadingError = 360 - HeadingError;

            } else if (HeadingError < -180.0) {
                HeadingError = 360 + HeadingError;
            }

            intThingheading += HeadingError;
            derThingHeading = HeadingError - LastErrorHeading;
            HeadingPower = (HeadingError * pHeading) + (intThingheading * iHeading) + (derThingHeading * dHeading);
            LastErrorHeading = HeadingError;



            FrontRightPower = (DrivePower + StrafePower - HeadingPower) * Speed;
            FrontLeftPower = (DrivePower - StrafePower + HeadingPower) * Speed;
            BackRightPower = (DrivePower - StrafePower - HeadingPower) * Speed;
            BackLeftPower = (DrivePower + StrafePower + HeadingPower) * Speed;

            setDrivePower(FrontLeftPower,FrontRightPower,BackLeftPower,BackRightPower);


            telemetry.addData("Target X ",xTarget);
            telemetry.addData("Target Y ",yTarget);
            telemetry.addData("Target Heading ",headingTarget);

            telemetry.addData("Current X ",xCurrent);
            telemetry.addData("Current Y ",yCurrent);
            telemetry.addData("pinponit Heading ",pinp.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Current Heading ",headingCurrent);

            telemetry.addData("HeadingError",HeadingError);
            telemetry.addData("StrafeError",StrafeError);
            telemetry.addData("DriveError",DriveError);

            telemetry.update();



        }
        setDrivePower(0,0,0,0);

    }


    void setDrivePower(double fl, double fr, double bl, double br) {
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }

}
