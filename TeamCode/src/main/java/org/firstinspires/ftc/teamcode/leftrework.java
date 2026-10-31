package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
* FTC WIRES Autonomous Example for only vision detection using tensorflow and park
*/
@Autonomous(name = "sample time", group = "10392-Autonomous", preselectTeleOp = "1 driver no tune")
public class leftrework extends LinearOpMode {


   public static String TEAM_NAME = "ROLLING"; //TODO: Enter team Name
   public static int TEAM_NUMBER = 10392; //TODO: Enter team Number
   private Servo servR, bar, sweep, sweep2;
   private CRServo servI;
   private DcMotor armT, armV, vL, vR;
   private VoltageSensor voltS;
   private double power;
   private double intakeP = -1, outakeP = .4;

   //Define and declare Robot Starting Locations
   public enum START_POSITION {
       LEFT
   }

   public static START_POSITION startPosition = START_POSITION.LEFT;


   @Override
   public void runOpMode() throws InterruptedException {


       //Key Pad input to selecting Starting Position of robot


       telemetry.update();


       armT = hardwareMap.get(DcMotorEx.class, "armM");
       armV = hardwareMap.get(DcMotorEx.class, "vipeM");
       servR = hardwareMap.get(Servo.class, "servR");      //rotation servo
       servI = hardwareMap.get(CRServo.class, "servI");
       bar = hardwareMap.get(Servo.class, "bar");
       voltS = hardwareMap.voltageSensor.iterator().next();


       sweep = hardwareMap.get(Servo.class, "sweep");
       sweep2 = hardwareMap.get(Servo.class, "sweep2");

       armT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       armV.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       armT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       armV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       armV.setDirection(DcMotor.Direction.REVERSE);
       armT.setDirection(DcMotor.Direction.REVERSE);

       waitForStart();


       //Game Play Button  is pressed
       if (opModeIsActive() && !isStopRequested()) {
           //Build parking trajectory based on last detected target by vision
           runAutonoumousMode();
       }
   }   // end runOpMode()


   public void runAutonoumousMode() {
       //Auto Left Positions - Samples
       Pose2d initPose = new Pose2d(0, 0, Math.toRadians(-90)); // Starting Pose
       Pose2d netZone = new Pose2d(16, 21, Math.toRadians(-45));
       Pose2d netZonec = new Pose2d(9, 21, Math.toRadians(-45));

       // x= y??? and y=x 8.2       7.1 17.4
       Pose2d yellowSampleOne = new Pose2d(29.45, -17, Math.toRadians(55));
       Pose2d yellowSampleOnec = new Pose2d(30.65, -1, Math.toRadians(55));

       Pose2d yellowSampleTwo = new Pose2d(32.8, -14.2, Math.toRadians(65));      //24
       Pose2d yellowSampleTwoc = new Pose2d(34.8, -2, Math.toRadians(65));

       //24
       Pose2d yellowSampleThree = new Pose2d(47, -9, Math.toRadians(90));
       Pose2d yellowSampleThree2 = new Pose2d(47, 6, Math.toRadians(90));

       Pose2d preSubmersiblePark = new Pose2d(51, 11, Math.toRadians(-90));
       Pose2d submersiblePark = new Pose2d(58, -14, Math.toRadians(-90));
       Pose2d submersiblePark2 = new Pose2d(58, 10, Math.toRadians(-90));
       ;
       Pose2d netZone2 = new Pose2d(16, 15, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4
       Pose2d netZone2c = new Pose2d(2, 15, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4

       Pose2d netZone3 = new Pose2d(14, 15, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4
       Pose2d netZone3c = new Pose2d(6, 15, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4

       Pose2d netZone4 = new Pose2d(3.5, 14, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4
       Pose2d netZone4c = new Pose2d(-7, 12, Math.toRadians(-45));       // x= y??? and y=x 8.2       7.1 17.4


       double waitSecondsBeforeDrop = 0;
       MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);


       if (startPosition == START_POSITION.LEFT) {
           servR.setPosition(.74);
           safeWaitSeconds(.4);
           Thread armThread = new Thread(() -> angle(newV(), 800));
           Thread vipeThread = new Thread(() -> vipe(newV(), 1050));
           Thread barThread = new Thread(() -> {
               intake(1);
               safeWaitSeconds(2);
               bar.setPosition(0);

           });
           armThread.start();
           vipeThread.start();
           barThread.start();


           //Move robot to netZone with preloaded sample ready to drop in basket
           Actions.runBlocking(
                   drive.actionBuilder(initPose)
                           .strafeToLinearHeading(netZone.position, netZone.heading)
                           .strafeToLinearHeading(netZonec.position, netZonec.heading)

                           .build());
           try {
               armThread.join();
               vipeThread.join();
               barThread.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }

           release(1);

           //Add code to drop sample in basket


           telemetry.addLine("Drop sample in basket");
           telemetry.update();
           servR.setPosition(.434);
           Thread armThread2 = new Thread(() -> {
               safeWaitSeconds(.2);
               angle(-newV(), 800);
           });
           Thread vipeThread2 = new Thread(() -> {
               safeWaitSeconds(.2);
               vipe(-newV(), 370);
           });
           armThread2.start();
           vipeThread2.start();
           //Move robot to pick yellow sample one
           Actions.runBlocking(
                   drive.actionBuilder(netZonec)
                           .strafeToLinearHeading(yellowSampleOne.position, yellowSampleOne.heading)
                           .build());
           try {
               armThread2.join();
               vipeThread2.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }

           Thread intakeThread = new Thread(() -> {
               servI.setPower(intakeP);
           });
           intakeThread.start();
           Actions.runBlocking(
                   drive.actionBuilder(yellowSampleOne)
                           .strafeToLinearHeading(yellowSampleOnec.position, yellowSampleOnec.heading)
                           .build());
           try {
               intakeThread.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }


           Thread armThread3 = new Thread(() -> angle(newV(), 1200));
           Thread vipeThread3 = new Thread(() -> {
               bar.setPosition(0);
               safeWaitSeconds(2);
               vipe(newV(), 620);
           });
           armThread3.start();
           vipeThread3.start();
           Actions.runBlocking(
                   drive.actionBuilder(yellowSampleOnec)
                           .strafeToLinearHeading(netZone2.position, netZone2.heading)
                           .strafeToLinearHeading(netZone2c.position, netZone2c.heading)

                           .build());
           try {
               armThread3.join();
               vipeThread3.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
           servR.setPosition(.74);
           safeWaitSeconds(.4);
           release(1);

           //Move robot to yellow sample two
           Thread armThread4 = new Thread(() -> {
               servR.setPosition(.434);

               safeWaitSeconds(.45);
               angle(-newV(), 800);
           });
           Thread vipeThread4 = new Thread(() -> {
               safeWaitSeconds(.4);
               vipe(-newV(), 300);
           });
           armThread4.start();
           vipeThread4.start();
           Actions.runBlocking(
                   drive.actionBuilder(netZone2c)
                           .strafeToLinearHeading(yellowSampleTwo.position, yellowSampleTwo.heading)
                           .build());
           try {
               armThread4.join();
               vipeThread4.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
           Thread intakeThread2 = new Thread(() -> {
               servI.setPower(intakeP);
           });
           intakeThread.start();
           Actions.runBlocking(
                   drive.actionBuilder(yellowSampleTwo)
                           .strafeToLinearHeading(yellowSampleTwoc.position, yellowSampleTwoc.heading)
                           .build());
           try {
               intakeThread2.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
           Thread armThread5 = new Thread(() -> angle(newV(), 1250));
           Thread vipeThread5 = new Thread(() -> {
               safeWaitSeconds(1.8);
               vipe(newV(), 520);
           });
           armThread5.start();
           vipeThread5.start();
           Actions.runBlocking(
                   drive.actionBuilder(yellowSampleTwoc)
                           .strafeToLinearHeading(netZone3.position, netZone3.heading)
                           .strafeToLinearHeading(netZone3c.position, netZone3c.heading)

                           .build());
           try {
               armThread5.join();
               vipeThread5.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
           servR.setPosition(.74);
           safeWaitSeconds(.3);
           release(.4);


           Thread armThread6 = new Thread(() -> {
               safeWaitSeconds(.6);
               angle(-newV(), 800);
           });
           Thread vipeThread6 = new Thread(() -> {
               safeWaitSeconds(.4);
               vipe(-newV(), 300);
           });
           armThread6.start();
           vipeThread6.start();
           Actions.runBlocking(
                   drive.actionBuilder(netZone3c)
                           .strafeToLinearHeading(yellowSampleThree.position, yellowSampleThree.heading)
                           .build());

           try {
               armThread6.join();
               vipeThread6.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
           Thread intakeThread3 = new Thread(() -> {
               servR.setPosition(.434);
               servI.setPower(intakeP);
           });
           intakeThread3.start();
           Actions.runBlocking(
                   drive.actionBuilder(yellowSampleThree)
                           .strafeToLinearHeading(yellowSampleThree2.position, yellowSampleThree2.heading)
                           .build());
           try {
               intakeThread3.join();
           } catch (InterruptedException e) {
               e.printStackTrace();
           }
               Thread vipeThread7 = new Thread(() -> {
                   servR.setPosition(.74);

                   safeWaitSeconds(1.8);
                   vipe(newV(), 620);
               });
           Thread armThread7 = new Thread(() -> angle(newV(), 1250));

           armThread7.start();
               vipeThread7.start();
               Actions.runBlocking(
                       drive.actionBuilder(yellowSampleThree)
                               .strafeToLinearHeading(netZone4.position, netZone4.heading)
                               .build());
               try {
                   armThread7.join();
                   vipeThread7.join();
               } catch (InterruptedException e) {
                   e.printStackTrace();
               }
               release(.4);
               Thread armThread10 = new Thread(() -> {
                   safeWaitSeconds(.4);
                   angle(-newV(), 800);
               });
               Thread vipeThread10 = new Thread(() -> {
                   safeWaitSeconds(.5);
                   vipe(-newV(), 1700);
               });
               armThread10.start();
               vipeThread10.start();

               Actions.runBlocking(
                       drive.actionBuilder(netZone4)
                               .strafeToLinearHeading(preSubmersiblePark.position, preSubmersiblePark.heading)
                               .build());

               try {
                   armThread10.join();
                   vipeThread10.join();
               } catch (InterruptedException e) {
                   e.printStackTrace();
               }


       /*    Actions.runBlocking(
                   drive.actionBuilder(preSubmersiblePark)
                           .strafeToLinearHeading(submersiblePark.position, submersiblePark.heading)
                           .build());*/

           }


       }


       //method to wait safely with stop button working if needed. Use this instead of sleep
       public void safeWaitSeconds ( double time){
           ElapsedTime timer = new ElapsedTime(SECONDS);
           timer.reset();
           while (!isStopRequested() && timer.time() < time) {
           }
       }
       public void intake (double time)
       {
           servI.setPower(intakeP);
           safeWaitSeconds(time);
           servI.setPower(0);
       }
       public void release ( double time)
       {
           servI.setPower(outakeP);
           safeWaitSeconds(time);
           servI.setPower(0);
       }
       public double newV ()
       {
           double battV = voltS.getVoltage();
           double desV = 12.0;
           double power = desV / battV;
           return power;
       }
       public void angle ( double power, long time){
           armT.setPower(power); // Set motor power to move towards the target position
           sleep(time);
           armT.setPower(0);
       }
       public void vipe (double power, int time)
       {
           armV.setPower(power);
           sleep(time);
           armV.setPower(0);


       }
   }   // end class
