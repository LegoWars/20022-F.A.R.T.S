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



    //Drive Config
    private PIDController DriveController;
    private PIDController StrafeController;


    public static double pDrive;
    public static double iDrive;
    public static double dDrive;

    public static double pStrafe;
    public static double iStrafe;
    public static double dStrafe;

    public static double pHeading;
    public static double iHeading;
    public static double dHeading;
    public double drivepid;
    public double strafepid;
    public double headingpid;
    public double drivevelocity;
    public double strafevelocity;
    public double headingvelocity;

    public static double speed;
    public double DefaultSpeed;
    public static double Z_Weight;

    public static double xTarget;
    public static double yTarget;
    public static double PrexTarget;
    public static double PreyTarget;
    public static double headingTarget;


    double laserrorheading;
    double intThingheading;
    public double headingerror;
    double laserrordrive;
    double IntThingdrive;
    double driveerror;


    public double xCurrent;
    public double yCurrent;
    public double headingCurrent;

    public double FrontRightTargetVelocity;
    public double FrontLeftTargetVelocity;
    public double BackRightTargetVelocity;
    public double BackLeftTargetVelocity;
    public double FrontRightTargetPower;
    public double FrontLeftTargetPower;
    public double BackRightTargetPower;
    public double BackLeftTargetPower;

    public Vector2d PureErrorVector = new Vector2d(0,0);
    public Vector2d PureTargetVector = new Vector2d(0,0);
    public Vector2d TargetPath;
    public Vector2d PastPath;
    public double TargetPathSqurd;
    public double ScalarProj;
    public Vector2d PointOnPath;
    public Vector2d ToPath;
    public Vector2d TrueError;
    public double Strict;
    public double Eager;

    public Pose2D startpose;

    public DcMotorEx FrontRight;
    public DcMotorEx FrontLeft;
    public DcMotorEx BackRight;
    public DcMotorEx BackLeft;


    GoBildaPinpointDriver odo;
    public double odoYawScalor;
    public double odoXoffSet;
    public double odoYoffSet;
    public int odoCalabrasionSleep;

    public VoltageSensor ControlHub_VoltageSensor;
    public VoltageSensor ExpansionHub2_VoltageSensor;

    private LinearOpMode myOpMode;

    public Kitchenet(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();



    //Personal Config
    private PIDController CapstanController;
    private PIDController SlidesController;
    public static double pCapstan;
    public static double iCapstan;
    public static double dCapstan;
    public static double fCapstan;

    public static double pSlides;
    public static double iSlides;
    public static double dSlides;
    public static double fSlides;
    public double Capstanpid;
    public double Slidespid;
    public double Capstanff;
    public double Capstanpower;
    public double Slidespower;

    public double CapstanTicksPerDegree;
    public static double CapstanTarget;
    public static double SlidesTarget;
    public double CapstanCurrent;
    public double SlidesCurrent;
    public double Capstan1Power;
    public double Capstan2Power;
    public double Slides1Power;
    public double Slides2Power;
    public DcMotorEx Capstan1;
    public DcMotorEx Capstan2;
    public DcMotorEx Slides1;
    public DcMotorEx Slides2;

    private Servo Servo2;
    private Servo Servo3;
    private Servo Servo1;



    public void initializeAuto(){

    //Drive Init

        laserrorheading = 0;
        intThingheading = 0;
        laserrordrive = 0;
        IntThingdrive = 0;
        driveerror = 0;

        pDrive = 90;
        iDrive = 0;
        dDrive = 4;

        pStrafe = 70;
        iStrafe = 0;
        dStrafe = 4;

        pHeading = 40;
        iHeading = 0;
        dHeading = 0;

        odoYawScalor = 1.0012049;
        odoXoffSet = -57.15;
        odoYoffSet = -152.4;
        odoCalabrasionSleep = 500;

        Eager = 1;
        Strict = 1;

        DefaultSpeed = 1;


        FrontLeft = MotorMap("FrontLeft", DcMotorEx.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight = MotorMap("FrontRight", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft = MotorMap( "BackLeft", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight = MotorMap( "BackRight",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        DriveController = new PIDController(pDrive,iDrive,dDrive);
        StrafeController = new PIDController(pStrafe,iStrafe,dStrafe);

        odo.setOffsets(odoXoffSet, odoYoffSet);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setYawScalar(odoYawScalor);

        odo.recalibrateIMU();
        myOpMode.sleep(odoCalabrasionSleep);

        if (startpose != null) {
            odo.setPosition(startpose);
        } else {
            xTarget = 0;
            yTarget = 0;
            PrexTarget = 0;
            PreyTarget = 0;
            headingTarget = 0;
        }

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();


        //Personal Init

        pCapstan = 0.002;
        iCapstan = 0;
        dCapstan = 0.0001;
        fCapstan = 0.0;

        CapstanTicksPerDegree = 8192/360;

        pSlides = 0.005;
        iSlides = 0.0;
        dSlides = 0.00001;
        fSlides = 0.0;

        CapstanTarget = 0;
        SlidesTarget = 0;

        Capstan1 = MotorMap( "Capstan1",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Capstan2 = MotorMap( "Capstan2",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides1 = MotorMap( "Slides1",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides2 = MotorMap( "Slides2",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo3 = ServoSetup("Servo3", Servo.Direction.FORWARD);
        Servo1 = ServoSetup("Servo1", Servo.Direction.FORWARD);
        Servo2 = ServoSetup("Servo2", Servo.Direction.FORWARD);

        CapstanController = new PIDController(pCapstan, iCapstan, dCapstan);
        SlidesController = new PIDController(pSlides, iSlides, dSlides);

    }



    public void initializeTele(){

    //Drive Init

        Z_Weight = 0.75;

        odoYawScalor = 1.0012049;
        odoXoffSet = -57.15;
        odoYoffSet = -152.4;
        odoCalabrasionSleep = 500;

        FrontLeft = MotorMap("FrontLeft", DcMotorEx.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight = MotorMap("FrontRight", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft = MotorMap( "BackLeft", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight = MotorMap( "BackRight",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ControlHub_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        ExpansionHub2_VoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(odoXoffSet, odoYoffSet);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setYawScalar(odoYawScalor);

        odo.recalibrateIMU();
        myOpMode.sleep(odoCalabrasionSleep);

        odo.setPosition(startpose);

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();


    //Personal Init

        CapstanTarget = 0;
        SlidesTarget = 0;

        pCapstan = 0.002;
        iCapstan = 0;
        dCapstan = 0.0001;
        fCapstan = 0.0;

        CapstanTicksPerDegree = 8192/360;

        pSlides = 0.005;
        iSlides = 0.0;
        dSlides = 0.00001;
        fSlides = 0.0;

        Capstan1 = MotorMap( "Capstan1",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Capstan2 = MotorMap( "Capstan2",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides1 = MotorMap( "Slides1",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides2 = MotorMap( "Slides2",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CapstanController = new PIDController(pCapstan, iCapstan, dCapstan);
        SlidesController = new PIDController(pSlides, iSlides, dSlides);

    }


    public void DriveUpdateTele(double xstick, double ystick, double headingstick, double speed) {


        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }

        double X = xstick;
        double Y = ystick;
        double Z = headingstick * Z_Weight;

        FrontLeftTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + Z));
        BackLeftTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + Z));
        FrontRightTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -(Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1)) + -Z));
        BackRightTargetPower = (speed * (X * Math.min(Math.max(1.4 * Math.cos((headingCurrent + 45) / 180 * Math.PI), -1), 1) + Y * Math.min(Math.max(1.4 * Math.cos((headingCurrent + -45) / 180 * Math.PI), -1), 1) + -Z));

        FrontRight.setPower(FrontRightTargetPower);
        FrontLeft.setPower(FrontLeftTargetPower);
        BackRight.setPower(BackRightTargetPower);
        BackLeft.setPower(BackLeftTargetPower);


    }


    public void PersonalUpdateTele(){


        CapstanController.setPID(pCapstan, iCapstan, dCapstan);
        SlidesController.setPID(pSlides, iSlides, dSlides);

        CapstanCurrent = Capstan1.getCurrentPosition();
        SlidesCurrent = Slides1.getCurrentPosition();

        Capstanpid = CapstanController.calculate(CapstanCurrent, CapstanTarget);
        Slidespid = SlidesController.calculate(SlidesCurrent, SlidesTarget);

        Capstanff = Math.cos(Math.toRadians(CapstanTarget / CapstanTicksPerDegree)) * fCapstan;

        Capstanpower = Capstanpid + Capstanff;
        Slidespower = Slidespid;

        if(Capstanpower < 0){
            Capstanpower /= 10 ;
        }

        Capstan1Power = Capstanpower;
        Capstan2Power = Capstanpower;
        Slides1Power = Slidespower;
        Slides2Power = Slidespower;

        Capstan1.setPower(Capstan1Power);
        Capstan2.setPower(Capstan2Power);
        Slides1.setPower(Slides1Power);
        Slides2.setPower(Slides2Power);

    }



    public void DriveUpdateAuto() {


        odo.update();
        Pose2D CurrentLocation = odo.getPosition();

        xCurrent = CurrentLocation.getX(DistanceUnit.INCH);
        yCurrent = CurrentLocation.getY(DistanceUnit.INCH);
        headingCurrent = -CurrentLocation.getHeading(AngleUnit.DEGREES);

        if (headingCurrent < 0) {
            headingCurrent = headingCurrent + 360;
        }

        TargetPath = new Vector2d((xTarget - PrexTarget),(yTarget - PreyTarget));
        PastPath = new Vector2d((xCurrent-PrexTarget),(yCurrent-PreyTarget));
        TargetPathSqurd = (TargetPath.getX() * TargetPath.getX()) + (TargetPath.getY() * TargetPath.getY());
        ScalarProj = PastPath.dot(TargetPath) / TargetPathSqurd;
        PointOnPath = (new Vector2d(PrexTarget,PreyTarget)).plus(TargetPath.times(ScalarProj));
        ToPath = PointOnPath.minus(new Vector2d(xCurrent,yCurrent));

        PureErrorVector = new Vector2d(xTarget-xCurrent,yTarget-yCurrent);
        PureTargetVector = PureErrorVector.rotateBy(-CurrentLocation.getHeading(AngleUnit.DEGREES));

        TrueError = (ToPath.times(Strict)).plus(PureTargetVector.times(Eager));

        drivepid = DrivePID(pDrive, iDrive, dDrive, TrueError.getX());
        strafepid = DrivePID(pStrafe, iStrafe, dStrafe, TrueError.getY());
        headingpid =HeadingPID(pHeading,iHeading,dHeading, headingCurrent, headingTarget);

        drivevelocity = drivepid;
        strafevelocity = strafepid;
        headingvelocity = headingpid;

        FrontRightTargetVelocity = (drivevelocity + strafevelocity - headingvelocity) * speed;
        FrontLeftTargetVelocity = (drivevelocity - strafevelocity + headingvelocity) * speed;
        BackRightTargetVelocity = (drivevelocity - strafevelocity - headingvelocity) * speed;
        BackLeftTargetVelocity = (drivevelocity + strafevelocity + headingvelocity) * speed;

        FrontRight.setVelocity(FrontRightTargetVelocity);
        FrontLeft.setVelocity(FrontLeftTargetVelocity);
        BackRight.setVelocity(BackRightTargetVelocity);
        BackLeft.setVelocity(BackLeftTargetVelocity);

    }


    public void PersonalUpdateAuto() {

        CapstanController.setPID(pCapstan, iCapstan, dCapstan);
        SlidesController.setPID(pSlides, iSlides, dSlides);

        CapstanCurrent = Capstan1.getCurrentPosition();
        SlidesCurrent = Slides1.getCurrentPosition();

        Capstanpid = CapstanController.calculate(CapstanCurrent, CapstanTarget);
        Slidespid = SlidesController.calculate(SlidesCurrent, SlidesTarget);
        Capstanff = Math.cos(Math.toRadians(CapstanTarget / CapstanTicksPerDegree)) * fCapstan;

        Capstanpower = Capstanpid + Capstanff;
        Slidespower = Slidespid;

        if(Capstanpower < 0){
            Capstanpower /= 10 ;
        }

        Capstan1Power = Capstanpower;
        Capstan2Power = Capstanpower;
        Slides1Power = Slidespower;
        Slides2Power = Slidespower;

        Capstan1.setPower(Capstan1Power);
        Capstan2.setPower(Capstan2Power);
        Slides1.setPower(Slides1Power);
        Slides2.setPower(Slides2Power);

    }


        DcMotorEx MotorMap(String deviceName, DcMotorEx.Direction direction, DcMotor.RunMode EncoderMode) {
            DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
            aMotor.setDirection(direction);
            aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
            aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aMotor.setMode(EncoderMode);  // Requires motor encoder cables to be hooked up.
            return aMotor;
        }


        Servo ServoSetup(String deviceName, Servo.Direction direction) {
            Servo aServo = myOpMode.hardwareMap.get(Servo.class, deviceName);
            aServo.setDirection(direction);
            return aServo;
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


    //Drive Telem
        myOpMode.telemetry.addLine("FARTS Data");

        myOpMode.telemetry.addData("Xerror ", PureErrorVector.getX());
        myOpMode.telemetry.addData("Yerror ", PureErrorVector.getY());
        myOpMode.telemetry.addData("headingerror ",headingerror);
        myOpMode.telemetry.addData("XTarget Vec ", TrueError.getX());
        myOpMode.telemetry.addData("YTarget Vec ", TrueError.getY());

        myOpMode.telemetry.addData("FrontRight ", FrontRight.getVelocity());
        myOpMode.telemetry.addData("FrontLeft ", FrontLeft.getVelocity());
        myOpMode.telemetry.addData("BackRight ", BackRight.getVelocity());
        myOpMode.telemetry.addData("BackLeft ", BackLeft.getVelocity());

        myOpMode.telemetry.addData("FrontRight Target ", FrontRightTargetVelocity);
        myOpMode.telemetry.addData("FrontLeft Target ", FrontLeftTargetVelocity);
        myOpMode.telemetry.addData("BackRight Target ", BackRightTargetVelocity);
        myOpMode.telemetry.addData("BackLeft Target ", BackLeftTargetVelocity);

        myOpMode.telemetry.addData("Target X ",xTarget);
        myOpMode.telemetry.addData("Target Y ",yTarget);

        myOpMode.telemetry.addData("Current X ",xCurrent);
        myOpMode.telemetry.addData("Current Y ",yCurrent);
        myOpMode.telemetry.addData("Current Heading ",headingCurrent);
        myOpMode.telemetry.addData("Speed ",speed);

        myOpMode.telemetry.addData("Battery Voltage", ControlHub_VoltageSensor.getVoltage());

        myOpMode.telemetry.addData("yaw scalor", odo.getYawScalar());


        TelemetryPacket packet1 = new TelemetryPacket();
        packet1.fieldOverlay()
                .drawGrid(0, 0, 144, 144, 7, 7)
                .setStroke("goldenrod")
                .strokeCircle(xCurrent, yCurrent, 4)
                .setStroke("blue")
                .strokeCircle(xTarget, yTarget, 4);


    //Personal Telem

        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addLine("Other Data");
        myOpMode.telemetry.addData("Target Heading ",headingTarget);
        myOpMode.telemetry.addData("Target Capstan ", CapstanTarget);


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

            xTarget = startpose.getX(DistanceUnit.INCH);
            yTarget = startpose.getY(DistanceUnit.INCH);
            PrexTarget = startpose.getX(DistanceUnit.INCH);
            PreyTarget = startpose.getY(DistanceUnit.INCH);
            headingTarget = startpose.getHeading(AngleUnit.DEGREES);

        }


    public void ResetPinpoint (){
        myOpMode.sleep(odoCalabrasionSleep);
        odo.setYawScalar(odoYawScalor);
        odo.recalibrateIMU();
        myOpMode.sleep(odoCalabrasionSleep);

    }


        public void driveTo(double x, double y) {

            if (x!=xTarget || y!=yTarget){
                PrexTarget=xTarget;
                PreyTarget=yTarget;
            }

            xTarget = x;
            yTarget = y;
            speed = DefaultSpeed;

        }

        public void driveTo(double x, double y, double heading) {

            if (x!=xTarget || y!=yTarget){
                PrexTarget=xTarget;
                PreyTarget=yTarget;
            }

            xTarget = x;
            yTarget = y;
            headingTarget = heading;
            speed = DefaultSpeed;

        }

        public void driveTo(double x, double y, double heading, double Speed) {

            if (x!=xTarget || y!=yTarget){
                PrexTarget=xTarget;
                PreyTarget=yTarget;
            }

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


        public void ResetCapstan (){
            Capstan1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Capstan2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void ResetSlides (){
            Slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void Servo1Position (double Position){
            Servo1.setPosition(Position);
        }

        public void Servo2Position (double Position){
            Servo2.setPosition(Position);
        }

        public void Servo3Position (double Position){
            Servo3.setPosition(Position);
        }


}