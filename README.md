# FTC 2025–2026 Robot Code

Robot code base for my FIRST Tech Challenge team, built on the official FTC SDK with [Road Runner 1.0](https://rr.brott.dev/) for motion planning and the FTC Wires autonomous and TeleOp starter.

**Built with:** Java · FTC SDK · Road Runner 1.0 · Android Studio

## What's here

| Path | What it is |
|---|---|
| `TeamCode/.../FTCWiresAutoIntoTheDeep.java` | Autonomous: pick a left or right start on the gamepad, then follow Road Runner trajectories |
| `TeamCode/.../FTCWiresTeleOpMode.java` | Robot-centric mecanum (or tank) TeleOp at half speed, with live pose telemetry |
| `TeamCode/.../MecanumDrive.java` | Road Runner mecanum drive: kinematics, feedforward and path following |
| `TeamCode/.../TwoDeadWheelLocalizer.java`, `ThreeDeadWheelLocalizer.java` | Odometry from dead-wheel encoders and the IMU |
| `TeamCode/.../tuning/` | Road Runner tuning OpModes: localization test, feedback tuner, spline test |
| `FtcRobotController/` | The FTC SDK and its sample OpModes |

## Status

Starter setup. The drive constants (`inPerTick`, `lateralInPerTick`, track width and feedback gains) still need to be tuned on the robot with the OpModes in `tuning/`.

## Build and deploy

1. Open the project in Android Studio (Ladybug or newer).
2. Connect to the Control Hub over Wi-Fi or USB.
3. Run the `TeamCode` configuration to build and install it on the robot.
4. On the Driver Station, choose **FTC Wires IntoTheDeep** (autonomous) or **FTC Wires TeleOp**.

## Credits

FTC SDK © FIRST, BSD-3-Clause. Road Runner by Ryan Brott. Autonomous and TeleOp templates from FTC Wires.
