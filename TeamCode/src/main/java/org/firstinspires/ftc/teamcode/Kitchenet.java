package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
public class Kitchenet {

    private PIDController DriveController;
    private PIDController StrafeController;
    private PIDController CapstanController;
    private PIDController SlidesController;

    public static double pDrive = 90;
    public static double iDrive = 0;
    public static double dDrive = 4;
//    public static double fDrive = 0;
    public static double pStrafe = 70;
    public static double iStrafe = 0;
    public static double dStrafe = 4;
//    public static double fStrafe = 0;
    public static double pHeading = 40;
    public static double iHeading = 0;
    public static double dHeading = 0;
//    public static double fHeading = 0;
    public static double pCapstan = 0.002;
    public static double iCapstan = 0;
    public static double dCapstan = 0.0001;
    public static double fCapstan = 0.0;

    public static double pSlides = 0.005;
    public static double iSlides = 0.0;
    public static double dSlides = 0.00001;
    public static double fSlides = 0.0;

//    public final double driveticksPerDegree = 537.7/360;
    public final double CapstanTicksPerDegree = 8192/360;

    public static double speed;

    public static double xTarget;
    public static double yTarget;
    public static double headingTarget;
    public static double CapstanTarget;
    public static double SlidesTarget;

    double laserrorheading = 0;
    double intThingheading = 0;
    public double headingerror;
    double laserrordrive = 0;
    double IntThingdrive = 0;
    double driveerror = 0;
    public static double Z_Weight = 0.75;

    public double xCurrent;
    public double yCurrent;
    public double headingCurrent;
    public double CapstanCurrent;
    public double SlidesCurrent;

    public double KilometersTargetVelocity;
    public double MilesTargetVelocity;
    public double FriesTargetVelocity;
    public double NuggsTargetVelocity;
    public double KilometersTargetPower;
    public double MilesTargetPower;
    public double FriesTargetPower;
    public double NuggsTargetPower;
    public double JohnPower;
    public double BobPower;
    public double EmmaPower;
    public double EmmrPower;
    public Vector2d errorVector;
    public Vector2d TargetVector;

    public Pose2D startpose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);


    public DcMotorEx Kilometers;
    public DcMotorEx Miles;
    public DcMotorEx Fries;
    public DcMotorEx Nuggs;
    public DcMotorEx John;
    public DcMotorEx Bob;
    public DcMotorEx Emma;
    public DcMotorEx Emmr;

    private Servo Nathaner;
    private Servo Nathanest;
    private Servo Nathan;

    GoBildaPinpointDriver odo;

    public VoltageSensor ControlHub_VoltageSensor;
    public VoltageSensor ExpansionHub2_VoltageSensor;

    private LinearOpMode myOpMode;

    public Kitchenet(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();


    public void initializeAuto(){

        DriveController = new PIDController(pDrive,iDrive,dDrive);
        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);
//        HeadingController = new PIDController(pHeading,iHeading,dHeading);
        CapstanController = new PIDController(pCapstan, iCapstan, dCapstan);
        SlidesController = new PIDController(pSlides, iSlides, dSlides);

        Miles  = setupDriveMotorAuto("Miles", DcMotorEx.Direction.FORWARD);
        Kilometers = setupDriveMotorAuto("Kilometers", DcMotor.Direction.REVERSE);
        Nuggs  = setupDriveMotorAuto( "Nuggs", DcMotor.Direction.FORWARD);
        Fries = setupDriveMotorAuto( "Fries",DcMotor.Direction.REVERSE);
        John = setupCapstanMotor( "John",DcMotor.Direction.FORWARD);
        Bob = setupCapstanMotor( "Bob",DcMotor.Direction.REVERSE);
        Emma = setupCapstanMotor( "Emma",DcMotor.Direction.FORWARD);
        Emmr = setupCapstanMotor( "Emmr",DcMotor.Direction.FORWARD);

        Nathanest = myOpMode.hardwareMap.get(Servo.class, "Nathanest");
        Nathan = myOpMode.hardwareMap.get(Servo.class, "Nathan");
        Nathaner = myOpMode.hardwareMap.get(Servo.class, "Nathaner");

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");




        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-57.15, -152.4);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setYawScalar(1.0012049);

        odo.recalibrateIMU();
        myOpMode.sleep(500);

        odo.setPosition(startpose);




        xTarget = 0;
        yTarget = 0;
        headingTarget = 0;
        CapstanTarget = 0;
        SlidesTarget = 0;
        errorVector = new Vector2d(0,0);
        TargetVector = new Vector2d(0,0);



        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

    }





    public void initializeTele(){

//        DriveController = new PIDController(pDrive,iDrive,dDrive);
//        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);
        //HeadingController = new PIDController(pHeading,iHeading,dHeading);
        CapstanController = new PIDController(pCapstan, iCapstan, dCapstan);
        SlidesController = new PIDController(pSlides, iSlides, dSlides);

        Miles  = setupDriveMotorTele("Miles", DcMotorEx.Direction.FORWARD);
        Kilometers = setupDriveMotorTele("Kilometers", DcMotor.Direction.REVERSE);
        Nuggs  = setupDriveMotorTele( "Nuggs", DcMotor.Direction.FORWARD);
        Fries = setupDriveMotorTele( "Fries",DcMotor.Direction.REVERSE);
        John = setupCapstanMotor( "John",DcMotor.Direction.FORWARD);
        Bob = setupCapstanMotor( "Bob",DcMotor.Direction.REVERSE);
        Emma = setupCapstanMotor( "Emma",DcMotor.Direction.FORWARD);
        Emmr = setupCapstanMotor( "Emmr",DcMotor.Direction.FORWARD);

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");




        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-57.15, -152.4);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setYawScalar(1.0012049);


        odo.recalibrateIMU();
        myOpMode.sleep(500);

        odo.setPosition(startpose);



//        xTarget = 0;
//        yTarget = 0;
//        headingTarget = 0;
        CapstanTarget = 0;
        SlidesTarget = 0;
//        errorVector = new Vector2d(0,0);
//        TargetVector = new Vector2d(0,0);



        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

    }



    private DcMotorEx setupDriveMotorAuto(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    private DcMotorEx setupDriveMotorTele(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }


    private DcMotorEx setupCapstanMotor(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }




    public void controllerUpdateTele(double xstick, double ystick, double headingstick, double speed) {


        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

//        DriveController.setPID(pDrive, iDrive, dDrive);
//        StrafeController.setPID(pStrafe, iStrafe, dStrafe);
        //HeadingController.setPID(pHeading, iHeading, dHeading);
        CapstanController.setPID(pCapstan, iCapstan, dCapstan);
        SlidesController.setPID(pSlides, iSlides, dSlides);

//        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
//        yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
//        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);



        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }

        double X = -xstick;
        double Y = ystick;
        double Z = headingstick * Z_Weight;

        //            leftBack --> Nuggs
        //            leftFront --> Miles
        //            rightFront --> Kilometers
        //            rightBack -- > Fries


        MilesTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + Z));
        NuggsTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + Z));
        KilometersTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + -Z));
        FriesTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -Z));


        CapstanCurrent = John.getCurrentPosition();

        SlidesCurrent = Emma.getCurrentPosition();

//        errorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
//        TargetVector = errorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

//        double drivepid = DrivePID(pDrive, iDrive, dDrive, TargetVector.getX());
//        double strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TargetVector.getY());
//        double headingpid =HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);
        //double headingpid = HeadingController.calculate(headingCurrent, headingTarget);
        double Capstanpid = CapstanController.calculate(CapstanCurrent, CapstanTarget);
        double Slidespid = SlidesController.calculate(SlidesCurrent, SlidesTarget);

//        double driveff = Math.cos(Math.toRadians(xTarget/driveticksPerDegree)) * fDrive;
//        double strafeff = Math.cos(Math.toRadians(yTarget/driveticksPerDegree)) * fStrafe;
//        double headingff = Math.cos(Math.toRadians(headingTarget/driveticksPerDegree)) * fHeading;
        double Capstanff = Math.cos(Math.toRadians(CapstanTarget / CapstanTicksPerDegree)) * fCapstan;

//        double drivevelocity = drivepid;
//        double strafevelocity = strafepid;
//        double headingvelocity = headingpid;
        double Capstanpower = Capstanpid + Capstanff;
        double Slidespower = Slidespid;

        if(Capstanpower < 0){
            Capstanpower /= 10 ;
        }


//            KilometersTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
//            MilesTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
//            FriesTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
//            NuggsTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

            JohnPower = Capstanpower;
            BobPower = Capstanpower;
            EmmaPower = Slidespower;
            EmmrPower = Slidespower;

            Kilometers.setPower(KilometersTargetPower);
            Miles.setPower(MilesTargetPower);
            Fries.setPower(FriesTargetPower);
            Nuggs.setPower(NuggsTargetPower);
            John.setPower(JohnPower);
            Bob.setPower(BobPower);
            Emma.setPower(EmmaPower);
            Emmr.setPower(EmmrPower);




    }


    public void controllerUpdateAuto() {


        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

//        DriveController.setPID(pDrive, iDrive, dDrive);
//        StrafeController.setPID(pStrafe, iStrafe, dStrafe);
        //HeadingController.setPID(pHeading, iHeading, dHeading);
        CapstanController.setPID(pCapstan, iCapstan, dCapstan);
        SlidesController.setPID(pSlides, iSlides, dSlides);

        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
        yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);



        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }
        CapstanCurrent = John.getCurrentPosition();

        SlidesCurrent = Emma.getCurrentPosition();

        errorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
        TargetVector = errorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

        double drivepid = DrivePID(pDrive, iDrive, dDrive, TargetVector.getX());
        double strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TargetVector.getY());
        double headingpid =HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);
        //double headingpid = HeadingController.calculate(headingCurrent, headingTarget);
        double Capstanpid = CapstanController.calculate(CapstanCurrent, CapstanTarget);
        double Slidespid = SlidesController.calculate(SlidesCurrent, SlidesTarget);

//        double driveff = Math.cos(Math.toRadians(xTarget/driveticksPerDegree)) * fDrive;
//        double strafeff = Math.cos(Math.toRadians(yTarget/driveticksPerDegree)) * fStrafe;
//        double headingff = Math.cos(Math.toRadians(headingTarget/driveticksPerDegree)) * fHeading;
        double Capstanff = Math.cos(Math.toRadians(CapstanTarget / CapstanTicksPerDegree)) * fCapstan;

        double drivevelocity = drivepid;
        double strafevelocity = strafepid;
        double headingvelocity = headingpid;
        double Capstanpower = Capstanpid + Capstanff;
        double Slidespower = Slidespid;

        if(Capstanpower < 0){
            Capstanpower /= 10 ;
        }


        KilometersTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
        MilesTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
        FriesTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
        NuggsTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

        JohnPower = Capstanpower;
        BobPower = Capstanpower;
        EmmaPower = Slidespower;
        EmmrPower = Slidespower;

        Kilometers.setVelocity(KilometersTargetVelocity);
        Miles.setVelocity(MilesTargetVelocity);
        Fries.setVelocity(FriesTargetVelocity);
        Nuggs.setVelocity(NuggsTargetVelocity);
        John.setPower(JohnPower);
        Bob.setPower(BobPower);
        Emma.setPower(EmmaPower);
        Emmr.setPower(EmmrPower);




    }


    public double HeadingPID(double p, double i, double d, double current, double target){

        headingerror = target - current;
        if (headingerror > 180.0){
            headingerror = 360 - headingerror;

        } else if (headingerror < -180.0) {
            headingerror = 360 + headingerror;
        }
        intThingheading += headingerror;
        double derThing = headingerror - laserrorheading;

//        if (Math.abs(error) >= Math.PI){
//            error -= Math.PI * 2;
//        } else if (error < Math.PI){
//            error += Math.PI * 2;
//        }

        double output = (headingerror * p) + (intThingheading * i) + (derThing * d);
        laserrorheading = headingerror;

                return output;
    }

    public double DrivePID(double p, double i, double d, double error){

        driveerror = error;
        IntThingdrive += driveerror;
        double derThing = driveerror - laserrordrive;
        double output = (driveerror * p) + (IntThingdrive * i) + (derThing * d);
        laserrordrive = driveerror;

        return output;
    }



    public void telemetryupdate(){


        myOpMode.telemetry.addData("Xerror ",errorVector.getX());
        myOpMode.telemetry.addData("Yerror ",errorVector.getY());
        myOpMode.telemetry.addData("headingerror ",headingerror);
        myOpMode.telemetry.addData("XTarget Vec ",TargetVector.getX());
        myOpMode.telemetry.addData("YTarget Vec ",TargetVector.getY());

        myOpMode.telemetry.addData("Kilometers ",Kilometers.getVelocity());
        myOpMode.telemetry.addData("Miles ",Miles.getVelocity());
        myOpMode.telemetry.addData("Fries ",Fries.getVelocity());
        myOpMode.telemetry.addData("Nuggs ",Nuggs.getVelocity());
//        myOpMode.telemetry.addData("John -",John.getPower());
//        myOpMode.telemetry.addData("Bob -",Bob.getPower());
//        myOpMode.telemetry.addData("Capstan Current -",John.getCurrentPosition());

        myOpMode.telemetry.addData("Kilometers Target ",KilometersTargetVelocity);
        myOpMode.telemetry.addData("Miles Target ",MilesTargetVelocity);
        myOpMode.telemetry.addData("Fries Target ",FriesTargetVelocity);
        myOpMode.telemetry.addData("Nuggs Target ",NuggsTargetVelocity);
//        myOpMode.telemetry.addData("John Target -", JohnPower);
//        myOpMode.telemetry.addData("Bob Target -", BobPower);


        myOpMode.telemetry.addData("Target X ",xTarget);
        myOpMode.telemetry.addData("Target Y ",yTarget);
        myOpMode.telemetry.addData("Target Heading ",headingTarget);
        myOpMode.telemetry.addData("Target Capstan ", CapstanTarget);

        myOpMode.telemetry.addData("Current X ",xCurrent);
        myOpMode.telemetry.addData("Current Y ",yCurrent);
        myOpMode.telemetry.addData("Current Heading ",headingCurrent);
        myOpMode.telemetry.addData("Speed ",speed);
        myOpMode.telemetry.addData("Battery Voltage", ControlHub_VoltageSensor.getVoltage());

        myOpMode.telemetry.addData("yaw scalor", odo.getYawScalar());


        TelemetryPacket packet1 = new TelemetryPacket();
        packet1.fieldOverlay()
                .drawGrid(0, 0, 144, 144, 7, 7)
               // .setTranslation(6 * 12, 6 * 12)
                .setStroke("goldenrod")
                .strokeCircle(xCurrent, yCurrent, 4)
                .setStroke("blue")
                .strokeCircle(xTarget, yTarget, 4);

        dashboard.sendTelemetryPacket(packet1);

        myOpMode.telemetry.update();
    }




        public void startlocation(double x, double y, double heading){
        double rotation;
            if (heading < 180){
                rotation = -heading;
            } else {
                rotation = -(heading - 360);
            }
            startpose = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES, rotation);
        }
        public void driveTo(double x, double y) {
            xTarget = x;
            yTarget = y;
            speed = 1;

        }

        public void driveTo(double x, double y, double heading) {
            xTarget = x;
            yTarget = y;
            headingTarget = heading;
            speed = 1;

        }

        public void driveTo(double x, double y, double heading, double Speed) {
            xTarget = x;
            yTarget = y;
            headingTarget = heading;
            speed = Speed;
        }


        public void CapstanPosition(double target) {
            CapstanTarget = target;
        }
        public void CapstanDegree(double targetdegree) {

            CapstanTarget = Math.min(Math.max(targetdegree, 0), 95) * CapstanTicksPerDegree;
        }

        public void CapstanLow() {
            CapstanTarget = 0;
        }

        public void CapstanHighBasket() {
            CapstanTarget = 94 * CapstanTicksPerDegree;
        }

        public void CapstanFloor() {
        CapstanTarget = 20 * CapstanTicksPerDegree;
    }

        public void CapstanWall(){
            CapstanTarget = 27 * CapstanTicksPerDegree;
        }


    public void SlidesPosition(double target) {

            SlidesTarget = target;
        }

        public void SlidesRetracted (){
            SlidesTarget = 0;
        }

        public void SlidesHighExtend (){
            SlidesTarget = 1450;
        }

        public void ResetPinpoint (){
            myOpMode.sleep(250);
            odo.setYawScalar(1.0012049);
            odo.recalibrateIMU();
            myOpMode.sleep(250);

        }




        public void ResetCapstan (){
            John.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void ResetEmmaEmmmr (){
            Emma.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Emmr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void NathanPosition (double Position){
            Nathan.setPosition(Position);
        }

        public void NathanerPosition (double Position){
            Nathaner.setPosition(Position);
        }

        public void NathanestPosition (double Position){
            Nathanest.setPosition(Position);
        }


}
