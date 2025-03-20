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
public class Kitchen {

    private PIDController DriveController;
    private PIDController StrafeController;
    private PIDController JohnBobController;
    private PIDController EmmaEmmrController;

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
    public static double pJohnBob = 0.002;
    public static double iJohnBob = 0;
    public static double dJohnBob = 0.0001;
    public static double fJohnBob = 0.0;

    public static double pEmmaEmmr = 0.005;
    public static double iEmmaEmmr = 0.0;
    public static double dEmmaEmmr = 0.00001;
    public static double fEmmaEmmr = 0.0;

//    public final double driveticksPerDegree = 537.7/360;
    public final double JohnBobTicksPerDegree = 8192/360;

    public static double speed;

    public static double xTarget;
    public static double yTarget;
    public static double headingTarget;
    public static double JohnBobTarget;
    public static double EmmaEmmrTarget;

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
    public double JohnBobCurrent;
    public double EmmaEmmrCurrent;

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

    public Kitchen(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();


    public void initializeAuto(){

        DriveController = new PIDController(pDrive,iDrive,dDrive);
        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);
//        HeadingController = new PIDController(pHeading,iHeading,dHeading);
        JohnBobController = new PIDController(pJohnBob, iJohnBob, dJohnBob);
        EmmaEmmrController = new PIDController(pEmmaEmmr, iEmmaEmmr, dEmmaEmmr);

        Miles  = setupDriveMotorAuto("Miles", DcMotorEx.Direction.FORWARD);
        Kilometers = setupDriveMotorAuto("Kilometers", DcMotor.Direction.REVERSE);
        Nuggs  = setupDriveMotorAuto( "Nuggs", DcMotor.Direction.FORWARD);
        Fries = setupDriveMotorAuto( "Fries",DcMotor.Direction.REVERSE);
        John = setupJohnBobMotor( "John",DcMotor.Direction.FORWARD);
        Bob = setupJohnBobMotor( "Bob",DcMotor.Direction.REVERSE);
        Emma = setupJohnBobMotor( "Emma",DcMotor.Direction.FORWARD);
        Emmr = setupJohnBobMotor( "Emmr",DcMotor.Direction.FORWARD);

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
        JohnBobTarget = 0;
        EmmaEmmrTarget = 0;
        errorVector = new Vector2d(0,0);
        TargetVector = new Vector2d(0,0);



        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

    }





    public void initializeTele(){

//        DriveController = new PIDController(pDrive,iDrive,dDrive);
//        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);
        //HeadingController = new PIDController(pHeading,iHeading,dHeading);
        JohnBobController = new PIDController(pJohnBob, iJohnBob, dJohnBob);
        EmmaEmmrController = new PIDController(pEmmaEmmr, iEmmaEmmr, dEmmaEmmr);

        Miles  = setupDriveMotorTele("Miles", DcMotorEx.Direction.FORWARD);
        Kilometers = setupDriveMotorTele("Kilometers", DcMotor.Direction.REVERSE);
        Nuggs  = setupDriveMotorTele( "Nuggs", DcMotor.Direction.FORWARD);
        Fries = setupDriveMotorTele( "Fries",DcMotor.Direction.REVERSE);
        John = setupJohnBobMotor( "John",DcMotor.Direction.FORWARD);
        Bob = setupJohnBobMotor( "Bob",DcMotor.Direction.REVERSE);
        Emma = setupJohnBobMotor( "Emma",DcMotor.Direction.FORWARD);
        Emmr = setupJohnBobMotor( "Emmr",DcMotor.Direction.FORWARD);

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
        JohnBobTarget = 0;
        EmmaEmmrTarget = 0;
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


    private DcMotorEx setupJohnBobMotor(String deviceName, DcMotorEx.Direction direction) {
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
        JohnBobController.setPID(pJohnBob, iJohnBob, dJohnBob);
        EmmaEmmrController.setPID(pEmmaEmmr, iEmmaEmmr, dEmmaEmmr);

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


        JohnBobCurrent = John.getCurrentPosition();

        EmmaEmmrCurrent = Emma.getCurrentPosition();

//        errorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
//        TargetVector = errorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

//        double drivepid = DrivePID(pDrive, iDrive, dDrive, TargetVector.getX());
//        double strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TargetVector.getY());
//        double headingpid =HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);
        //double headingpid = HeadingController.calculate(headingCurrent, headingTarget);
        double johnbobpid = JohnBobController.calculate(JohnBobCurrent, JohnBobTarget);
        double emmaemmrpid = EmmaEmmrController.calculate(EmmaEmmrCurrent, EmmaEmmrTarget);

//        double driveff = Math.cos(Math.toRadians(xTarget/driveticksPerDegree)) * fDrive;
//        double strafeff = Math.cos(Math.toRadians(yTarget/driveticksPerDegree)) * fStrafe;
//        double headingff = Math.cos(Math.toRadians(headingTarget/driveticksPerDegree)) * fHeading;
        double johnbobff = Math.cos(Math.toRadians(JohnBobTarget/JohnBobTicksPerDegree)) * fJohnBob;

//        double drivevelocity = drivepid;
//        double strafevelocity = strafepid;
//        double headingvelocity = headingpid;
        double johnbobpower = johnbobpid + johnbobff;
        double emmaemmrpower = emmaemmrpid;

        if(johnbobpower < 0){
            johnbobpower /= 10 ;
        }


//            KilometersTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
//            MilesTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
//            FriesTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
//            NuggsTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

            JohnPower = johnbobpower;
            BobPower = johnbobpower;
            EmmaPower = emmaemmrpower;
            EmmrPower = emmaemmrpower;

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
        JohnBobController.setPID(pJohnBob, iJohnBob, dJohnBob);
        EmmaEmmrController.setPID(pEmmaEmmr, iEmmaEmmr, dEmmaEmmr);

        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
        yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);



        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }
        JohnBobCurrent = John.getCurrentPosition();

        EmmaEmmrCurrent = Emma.getCurrentPosition();

        errorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
        TargetVector = errorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

        double drivepid = DrivePID(pDrive, iDrive, dDrive, TargetVector.getX());
        double strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TargetVector.getY());
        double headingpid =HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);
        //double headingpid = HeadingController.calculate(headingCurrent, headingTarget);
        double johnbobpid = JohnBobController.calculate(JohnBobCurrent, JohnBobTarget);
        double emmaemmrpid = EmmaEmmrController.calculate(EmmaEmmrCurrent, EmmaEmmrTarget);

//        double driveff = Math.cos(Math.toRadians(xTarget/driveticksPerDegree)) * fDrive;
//        double strafeff = Math.cos(Math.toRadians(yTarget/driveticksPerDegree)) * fStrafe;
//        double headingff = Math.cos(Math.toRadians(headingTarget/driveticksPerDegree)) * fHeading;
        double johnbobff = Math.cos(Math.toRadians(JohnBobTarget/JohnBobTicksPerDegree)) * fJohnBob;

        double drivevelocity = drivepid;
        double strafevelocity = strafepid;
        double headingvelocity = headingpid;
        double johnbobpower = johnbobpid + johnbobff;
        double emmaemmrpower = emmaemmrpid;

        if(johnbobpower < 0){
            johnbobpower /= 10 ;
        }


        KilometersTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
        MilesTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
        FriesTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
        NuggsTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

        JohnPower = johnbobpower;
        BobPower = johnbobpower;
        EmmaPower = emmaemmrpower;
        EmmrPower = emmaemmrpower;

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
//        myOpMode.telemetry.addData("JohnBob Current -",John.getCurrentPosition());

        myOpMode.telemetry.addData("Kilometers Target ",KilometersTargetVelocity);
        myOpMode.telemetry.addData("Miles Target ",MilesTargetVelocity);
        myOpMode.telemetry.addData("Fries Target ",FriesTargetVelocity);
        myOpMode.telemetry.addData("Nuggs Target ",NuggsTargetVelocity);
//        myOpMode.telemetry.addData("John Target -", JohnPower);
//        myOpMode.telemetry.addData("Bob Target -", BobPower);


        myOpMode.telemetry.addData("Target X ",xTarget);
        myOpMode.telemetry.addData("Target Y ",yTarget);
        myOpMode.telemetry.addData("Target Heading ",headingTarget);
        myOpMode.telemetry.addData("Target JohnBob ",JohnBobTarget);

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


        public void JohnBobPosition(double target) {
            JohnBobTarget = target;
        }
        public void JohnBobDegree(double targetdegree) {

            JohnBobTarget = Math.min(Math.max(targetdegree, 0), 95) * JohnBobTicksPerDegree;
        }

        public void JohnBobLow() {
            JohnBobTarget = 0;
        }

        public void JohnBobHighBasket() {
            JohnBobTarget = 94 * JohnBobTicksPerDegree;
        }

        public void JohnBobFloor() {
        JohnBobTarget = 20 * JohnBobTicksPerDegree;
    }

        public void JohnBobWall(){
            JohnBobTarget = 27 * JohnBobTicksPerDegree;
        }


    public void EmmaEmmrPosition(double target) {

            EmmaEmmrTarget = target;
        }

        public void EmmaEmmrRetracted (){
            EmmaEmmrTarget = 0;
        }

        public void EmmaEmmrHighExtend (){
            EmmaEmmrTarget = 1450;
        }

        public void ResetPinpoint (){
            myOpMode.sleep(250);
            odo.setYawScalar(1.0012049);
            odo.recalibrateIMU();
            myOpMode.sleep(250);

        }




        public void ResetJohnBob (){
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
