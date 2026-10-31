# FTC Team 10392 Rolling: Robot Code

Robot code for FIRST Tech Challenge team **10392 Rolling**, written for the *INTO THE DEEP* game. It's built on the official FTC SDK, with [Road Runner 1.0](https://rr.brott.dev/) for autonomous motion and FTC Dashboard for live tuning.

**Built with:** Java · FTC SDK · Road Runner 1.0 · FTCLib · FTC Dashboard · Android Studio

## OpModes

### Autonomous: `sample time` ([`leftrework.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/leftrework.java))

A four-sample routine from the left start: score the preload in the basket, cycle the three yellow spike-mark samples into the net zone, then drive toward the submersible to park.

- **Parallel mechanisms.** The arm and viper slide move on their own threads while Road Runner drives, so the robot reaches the basket already raised instead of stopping to lift.
- **Battery compensation.** Arm and slide power is scaled by `12 V / battery voltage`, so timed moves land in the same place on a fresh or a drained battery.
- **Poses** for every pickup and drop are tuned by hand on the field (`netZone*`, `yellowSample*`).

### TeleOp: `1 driver no tune` ([`DRIVE1N.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DRIVE1N.java))

Single-driver TeleOp with closed-loop mechanism control.

- **PID plus feedforward** on the arm and viper slide (FTCLib `PIDController`), with a cosine gravity term based on arm angle. The gains are `@Config` fields, so they can be tuned live from FTC Dashboard.
- **Presets.** **A** stows (arm down, slide in, wrist level) and **B** raises to scoring height. **X** extends to intake in stowed mode, or to full basket height in scoring mode.
- **Auto homing.** When the arm comes down, a limit switch zeroes its encoder, so position drift doesn't build up over a match.
- **Intake and manipulators.** Continuous-rotation intake (**X** in, **Y** out), a wrist servo, a sweeper toggle (right stick click) and a bar toggle (gamepad 2 **Y**).
- **Precision mode.** Holding left bumper drops drive speed to 40%.
- **Hang mode.** Gamepad 2 **A** switches the drive to gamepad 2 and gives the first driver direct control of the arm, the slide and the two ascent motors (D-pad). Gamepad 2 **B** switches back.

### Road Runner setup

| Path | What it is |
|---|---|
| `MecanumDrive.java` | Road Runner mecanum drive: kinematics, feedforward and path following |
| `TwoDeadWheelLocalizer.java`, `ThreeDeadWheelLocalizer.java` | Odometry from dead-wheel encoders and the IMU |
| `tuning/` | Road Runner tuning OpModes: localization test, feedback tuner, spline test |
| `FTCWiresAutoIntoTheDeep.java`, `FTCWiresTeleOpMode.java` | The FTC Wires starter OpModes this code base grew from |

## Robot configuration

The OpModes expect these names in the Driver Station configuration:

| Name | Device |
|---|---|
| `leftFront`, `rightFront`, `leftRear`, `rightRear` | Drive motors |
| `armM` | Arm pivot motor |
| `vipeM` | Viper slide motor |
| `LH`, `RH` | Ascent (hang) motors |
| `servR` | Wrist rotation servo |
| `servI` | Intake (continuous rotation) |
| `bar`, `sweep`, `sweep2` | Bar and sweeper servos |
| `sensM` | Arm home limit switch (digital) |

## Build and deploy

1. Open the project in Android Studio.
2. Connect to the Control Hub over Wi-Fi or USB.
3. Run the `TeamCode` configuration to build and install it on the robot.
4. On the Driver Station, choose **sample time** (autonomous) or **1 driver no tune** (TeleOp).
5. To tune the arm and slide, open FTC Dashboard at `http://192.168.43.1:8080/dash` while the TeleOp runs and edit `p`, `f`, `p2` and `f2` under `DRIVE1N`.

## Credits

FTC SDK © FIRST, BSD-3-Clause. Road Runner by Ryan Brott. FTCLib. Starter templates from FTC Wires.
