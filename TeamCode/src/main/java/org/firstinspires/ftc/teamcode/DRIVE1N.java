package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;


@Config
@TeleOp(name = "1 driver no tune", group = "Examples")


public class DRIVE1N extends OpMode {


   // todo: write your code here
   private DcMotor frontleft, frontright;
   private DcMotor backleft, backright;
   private PIDController controller1, controller2;
   public static double p=0.001, i=0, d=0,p2=0.00003, i2=0, d2=0;
   public static double f=0.00043,f2=0.0005;
   public static int target =0,  target2 =0;
   int Adown=2000,Aup=-2660,Vstart=0,Vend, limit=-2200;
   private DigitalChannel sensM;
   private double intakeP=-1, outakeP=.4;
   private boolean track, armMode;
   private final double ticks_in_degree = (1993.6*.2)/180.0, ticks_in_degree2 = (384.5)/180.0;        //switch ratio
   private DcMotorEx arm_motor,vipe_motor;
   private Servo servR,bar,sweep,sweep2;
   private CRServo servI;
   private DcMotor LH,RH;
   private double servP=0.65;
   private double servPower;
   double driverspeed = 1;
   private boolean clicked,click;
   private boolean hangtime=false;


   Gamepad game2 = new Gamepad();
   Gamepad pgame2 = new Gamepad();
   Gamepad pgame = new Gamepad();
   Gamepad game = new Gamepad();
   boolean sweeptoggle;

   boolean bartoggle;


   @Override
   public void init() {
       frontleft = hardwareMap.get(DcMotor.class, "leftFront");
       frontright = hardwareMap.get(DcMotor.class, "rightFront");
       backright = hardwareMap.get(DcMotor.class, "rightRear");
       backleft = hardwareMap.get(DcMotor.class, "leftRear");


       controller1 = new PIDController(p, i, d);
       controller2 = new PIDController(Math.sqrt(p2), i2, d2);
       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


       arm_motor = hardwareMap.get(DcMotorEx.class, "armM");
       vipe_motor = hardwareMap.get(DcMotorEx.class, "vipeM");
       servR = hardwareMap.get(Servo.class, "servR");      //rotation servo
       servI = hardwareMap.get(CRServo.class, "servI");
       sensM = hardwareMap.get(DigitalChannel.class, "sensM");
       bar = hardwareMap.get(Servo.class,"bar");


       sweep= hardwareMap.get(Servo.class,"sweep");
       sweep2= hardwareMap.get(Servo.class,"sweep2");


       LH = hardwareMap.get(DcMotor.class, "LH");
       RH = hardwareMap.get(DcMotor.class, "RH");


       frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       vipe_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       vipe_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


   }
   public void loop()
   {
       if(game.y && !pgame.y)
       {
           bartoggle =!bartoggle;
       }
       if(bartoggle)
       {
           bar.setPosition(.5);
       }
       else {
           bar.setPosition(0);
       }

       if (gamepad2.a) {
           hangtime = true;
       }
       if (gamepad2.b) {
           hangtime = false;
       }
if(!hangtime) {
   servR.setPosition(servP);
   servI.setPower(servPower);
   controller1.setPID(p, i, d);
   controller2.setPID(Math.sqrt(p2), i2, d2);
   int armpos = arm_motor.getCurrentPosition();
   int vipepos = vipe_motor.getCurrentPosition();
   double pid = controller1.calculate(armpos, target);
   double pid2 = controller2.calculate(vipepos, target2);
   double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
   double ff2 = Math.cos(Math.toRadians(target / ticks_in_degree2)) * f2;
   double power = pid + ff;
   double power2 = pid2 + ff2;
   arm_motor.setPower(power);
   vipe_motor.setPower(power2);

   telemetry.addData("armpos", armpos);
   telemetry.addData("armtarget", target);
   telemetry.addData("pressed", sensM.getState());


   telemetry.addData("vipepos", vipepos);
   telemetry.addData("vipetarget", target2);


   telemetry.update();

}


       //driving


if(hangtime){
   arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   vipe_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   vipe_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   RH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   LH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   double max;
   // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
   double axial = -gamepad2.right_stick_x;  // Note: pushing stick forward gives negative value
   double lateral = gamepad2.left_stick_x;
   double yaw = gamepad2.left_stick_y;
   double leftFrontPower = axial - lateral + yaw;
   double rightFrontPower = axial - lateral - yaw;
   double leftBackPower = axial + lateral + yaw;
   double rightBackPower = axial + lateral - yaw;
   max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
   max = Math.max(max, Math.abs(leftBackPower));
   max = Math.max(max, Math.abs(rightBackPower));
   //arm
   if(gamepad1.left_stick_y>0)
   {
       arm_motor.setPower(1);
   }
   else if(gamepad1.left_stick_y<0)
   {
       arm_motor.setPower(-1);
   }
   else {
       arm_motor.setPower(0);
   }
   //viper
   if(gamepad1.left_trigger>0)
   {
       vipe_motor.setPower(1);
   }
   else if(gamepad1.right_trigger>0)
   {
       vipe_motor.setPower(-1);
   }
   else {
       vipe_motor.setPower(0);
   }
   //ascent 3 thingies
   if(gamepad1.dpad_down || gamepad2.dpad_down)//maybe add while loop to make it faster
   {
       LH.setPower(-1);
       RH.setPower(1);
   }
   else if(gamepad1.dpad_up || gamepad2.dpad_up)
   {
       LH.setPower(1);
       RH.setPower(-1);
   }
   else
   {
       LH.setPower(0);
       RH.setPower(0);
   }
   if (max > 1.0) {
       leftFrontPower /= max;
       rightFrontPower /= max;
       leftBackPower /= max;
       rightBackPower /= max;
   }
   frontleft.setPower(leftFrontPower * driverspeed);
   frontright.setPower(rightFrontPower * driverspeed);
   backleft.setPower(leftBackPower * driverspeed);
   backright.setPower(rightBackPower * driverspeed);
}

if(!hangtime) {
           double max;
           // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
           double axial = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
           double lateral = gamepad1.left_stick_x;
           double yaw = gamepad1.left_stick_y;
           double leftFrontPower = axial - lateral + yaw;
           double rightFrontPower = axial - lateral - yaw;
           double leftBackPower = axial + lateral + yaw;
           double rightBackPower = axial + lateral - yaw;
           max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
           max = Math.max(max, Math.abs(leftBackPower));
           max = Math.max(max, Math.abs(rightBackPower));
           if (max > 1.0) {
               leftFrontPower /= max;
               rightFrontPower /= max;
               leftBackPower /= max;
               rightBackPower /= max;
           }
           pgame2.copy(game2);
           pgame.copy(game);
           game2.copy(gamepad1);
           game.copy(gamepad2);

           if ((game2.right_stick_button && !pgame2.right_stick_button) || (game.x && !pgame.x)) {
               sweeptoggle = !sweeptoggle;
           }
           if (sweeptoggle) {
               sweep2.setPosition(.3);
               sweep.setPosition(.8);
           } else {
               sweep2.setPosition(.7);
               sweep.setPosition(.2);
           }

           frontleft.setPower(leftFrontPower * driverspeed);
           frontright.setPower(rightFrontPower * driverspeed);
           backleft.setPower(leftBackPower * driverspeed);
           backright.setPower(rightBackPower * driverspeed);


           // advanced arm and viper stuff lmao


           if (gamepad1.a) {
               target = Adown;
               servP = .65;
               armMode = false;
               target2 = Vstart;
               track = false;


           }
           if (gamepad1.b) {
               servP = .74;
               target = Aup;
               armMode = true;


           }


           if (!armMode) //down
           {


               if (!sensM.getState() && !track) {
                   arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                   track = true;
                   arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   target = -20;
                   Adown = 0;
               }


               if (gamepad1.y) {
                   servPower = outakeP;
               }
               if (gamepad1.x) {
                   target2 = -1100;
                   servP = .434;
                   servPower = intakeP;
               }
               if (gamepad1.left_trigger > 0) {
                   if (target2 >= 2000) {
                       vipe_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                       target=0;
                   } else {
                       target2 += 20;
                   }
               } else if (gamepad1.right_trigger > 0) {
                   if (target2 <= -1300) {
                       target2 += 0;
                   } else {
                       servP= .43;
                       target2 += -20;
                   }
               }
           }
           if (armMode) //up
           {
               //   p2=0.00003;
               //  f2=0.0005;


               if (gamepad1.right_trigger > 0) {
                   target2 += -20;
               } else if (gamepad1.left_trigger > 0) {
                   target2 += 20;
               }
               if (gamepad1.x) {
                   target2 = -1950;
               }
               if (gamepad1.y) {
                   servPower = outakeP;
               }


           }


           //servo time
           if (gamepad1.left_bumper) {
               driverspeed = .4;
               if (gamepad1.y) {
                   servPower = outakeP;
               }


               //servP=.4;
           } else if (gamepad1.right_bumper) {
               servP -= .01;
           } else {
               driverspeed = 1;
           }
           if(gamepad1.left_stick_button)
           {
               target=3000;
               target2=1000;
               if(gamepad1.a)
               {
                   arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                   vipe_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                   arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   vipe_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   target=Adown;
                   target2=Vstart;
               }
           }

       }
   }
}
