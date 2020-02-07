/*
Copyright 2019 FIRST Tech Challenge Team 12886

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope; 
import com.qualcomm.robotcore.hardware.Servo;
import java.text.DecimalFormat;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.SimpleDateFormat;
import java.util.Date;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class AutoAdvancedBlue extends OpMode {
    // CONFIGURATION

    // Expansion Hub 1:
    // Motors:
    // Port 0: manArm
    // Servos
    // Port 0: leHand <-- Still needed
    // Sensors:
    // I2C Bus 0: Expansion Hub IMU
    // I2C Bus 1: frontDistance

    // Expansion Hub 2:
    // Motors:
    // Port 0: frontLeft
    // Port 1: rearLeft
    // Port 2: frontRight
    // Port 3: rearRight

    public DecimalFormat format = new DecimalFormat("##.00");
    private BNO055IMU imu;

    private AndroidGyroscope Gyro; 

    private DcMotor manArm;
    private boolean rightBumperDown = false;
    private boolean leftBumperDown = false;
    private boolean rightStickClick = false;

    private Servo elbow;
    private Servo hand;
  
    private Servo leftFound;
    private Servo rightFound;

    private DistanceSensor frontDistance;
    private ColorSensor frontColor;
    private DistanceSensor frontRange;
    private DistanceSensor rearDist;
    private DistanceSensor leftDist;
    private DistanceSensor rightDist;

    private String armState = "0block"; // <-- Using string comparators to better explain...
                                        // ...what is happening with the armState
    private String driveState = "step1";
    private String autoState = "attackSkystone";

    public Orientation angles;
    public Acceleration gravity;

    private boolean arcadeMode = false;
    private boolean manualControl = false;

    private int gyroCalibratedCount = 0;
    private float manualArmPower;

    private MecanumAutonomousAdvanced mecanum;

    private ElapsedTime time;

        // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};

    private float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
    private double SCALE_FACTOR = 255;
    private View relativeLayout;
    private int relativeLayoutId;
    private int currentPos; 

    @Override
    public void init() {
      armState = "0block";
      motorInit();
      gyroInit();
      sensorInit();
      time = new ElapsedTime();
      time.reset();
    }

    @Override
    public void init_loop() {
      gyroLoop();
    }

    @Override
    public void loop() {
      telemetry.addData("autoState", autoState);
      gyroLoop();
      seeWorld();
      driveLoop();
      mecanum.go(); 
      time.reset();
    }

    private void motorInit() {
      mecanum = new MecanumAutonomousAdvanced();
      mecanum.init(gamepad1, hardwareMap);

      manArm = hardwareMap.get(DcMotor.class, "manArm");
      hand = hardwareMap.get(Servo.class, "leHand");

      leftFound = hardwareMap.get(Servo.class, "leftFound");
      rightFound = hardwareMap.get(Servo.class, "rightFound");

      manArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 

      telemetry.addData("MOTORS", "Initialized");
    }

    private void gyroInit() {
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

      parameters.mode                = BNO055IMU.SensorMode.IMU;
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.loggingEnabled      = false;

      imu.initialize(parameters);
    }

    private void sensorInit() {
      frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
      frontColor = hardwareMap.get(ColorSensor.class, "frontDistance");
      frontRange = hardwareMap.get(DistanceSensor.class, "frontDistyboi");
      rearDist = hardwareMap.get(DistanceSensor.class, "rearDist");
      leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
      rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");

      // values is a reference to the hsvValues array.
      float values[] = hsvValues;
    }

    private void gyroLoop() {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("HEADING", angles.firstAngle);
    }

    private void dropBlock() {
      if(gamepad1.left_bumper) {
        if(!leftBumperDown){
          leftBumperDown = true;
          hand.setPosition(0.3);
        }
      } else {
        leftBumperDown = false;
      }
    }

    private void seeWorld() {
      Color.RGBToHSV(
        (int) (frontColor.red() * SCALE_FACTOR),
        (int) (frontColor.green() * SCALE_FACTOR),
        (int) (frontColor.blue() * SCALE_FACTOR),
        hsvValues);
    }

    private void moveArm() {
      if(manualControl){
        manualMoveArm(); // <-- If manual control mode is active...
                          // ...revert to trigger control.
                          // RightTrigger = up
                          // LeftTrigger = down
        return;          // <-- This will exit the function early so...
                          // ...the state machine doesn't control the arm
      }

      switch(armState){
        case "0block":
        manArm.setTargetPosition(20);
        if(frontDistance.getDistance(DistanceUnit.CM) < 5.5 && !leftBumperDown) {
          hand.setPosition(1);
          if(hand.getPosition() == 1) {
            armState = "1block";
          }
        }
        break;

        case "1block":
        manArm.setTargetPosition(420);
        if(frontDistance.getDistance(DistanceUnit.CM) < 5.5 && !leftBumperDown) {
          hand.setPosition(1);
        }
        break;

        case "2block":
        manArm.setTargetPosition(600);
        if(frontDistance.getDistance(DistanceUnit.CM) < 5.5 && !leftBumperDown) {
          hand.setPosition(1);
        }
        break;

        case "3block":
        manArm.setTargetPosition(880);
        if(frontDistance.getDistance(DistanceUnit.CM) < 5.5 && !leftBumperDown) {
          hand.setPosition(1);
        }
        break;

        case "capstone":
        manArm.setTargetPosition(1140);
        if(frontDistance.getDistance(DistanceUnit.CM) < 5.5 && !leftBumperDown) {
          hand.setPosition(1);
        }
        break;
      }

      manArm.setPower(0.3);
      manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      telemetry.addData("ArcadeMode:", arcadeMode);
    }

    private void driveLoop() {
      switch (autoState){
        case "attackSkystone":
          stayOnTarget(2, -2);
          mecanum.setFwd(-0.8f);
          hand.setPosition(0.3);
          if(frontRange.getDistance(DistanceUnit.CM) < 25){
            autoState = "slipperySlide";
          }
        break;
        
        case "slipperySlide":
          if(hsvValues[0] > 100){
            mecanum.setSlide(0.0f);
            mecanum.setFwd(0.0f);
            autoState = "forcedHug";
          }
          else{
            stayOnTarget(2, -2);
            mecanum.setSlide(0.7f);
            mecanum.setFwd(0.2f);
          }
        break;
        
        case "forcedHug":
          moveArm(); 
          if(armState == "1block"){
            mecanum.setFwd(0.0f);
            mecanum.setSlide(0.0f); 
            autoState = "tacticalRetreat";
          } else {
            mecanum.setFwd(-0.5f);
          }
        break;  
        
        case "tacticalRetreat":
          moveArm();
          if(rearDist.getDistance(DistanceUnit.CM) > 20){
              stayOnTarget(2, -2);
              mecanum.setFwd(0.7f);
          } else {
              mecanum.setFwd(0.0f);
              autoState = "aaaaandSlide";
          } 
        break;   
           
        case "aaaaandSlide":
          moveArm();
          if(rightDist.getDistance(DistanceUnit.CM) > 60){
            mecanum.setFwd(0.0f); 
            mecanum.setSlide(0.0f);
            autoState = "spinnyBoi";
          } else {
            stayOnTarget(2, -2);
            mecanum.setSlide(-0.65f);
            mecanum.setFwd(0.05f); 
          } 
        break;
             
        case "spinnyBoi":
          moveArm();
          if(angles.firstAngle >= -70){
              mecanum.setRotate(0.5f);
          } else {
              mecanum.setRotate(0.0f);
              autoState = "chargeBoi";
          } 
        break;
            
        case "chargeBoi":
          moveArm();
          stayOnTarget(-88, -92); 
          if(mecanum.getFrontRight() < 3200){
              mecanum.setFwd(0.9f);
          } else if(rearDist.getDistance(DistanceUnit.CM) > 45){
              mecanum.setFwd(0.8f);
          } else {
              mecanum.setFwd(0.0f);
              autoState = "spinTowardsFound"; 
          }
        break;

        case "spinTowardsFound":
          moveArm(); 
          if(angles.firstAngle <= -20){
              mecanum.setRotate(-0.5f);
          } else {
              mecanum.setRotate(0.0f);
              autoState = "driveTowardsFound";
          }
        break; 

        case "driveTowardsFound":
          moveArm();
          stayOnTarget(2, -2); 
          if(rearDist.getDistance(DistanceUnit.CM) < 69){
              mecanum.setFwd(-0.85f);
          } else {
              mecanum.setFwd(0.0f);
              autoState = "lads"; 
          }
        break;
        
        case "lads":
          moveArm(); 
          hand.setPosition(0.3); 
          if(hand.getPosition() == 0.3){
            autoState = "juicyPoints"; 
          }
        break; 

        case "juicyPoints":
          moveArm();
          hand.setPosition(0.3); 
          if(angles.firstAngle >= -150){
            mecanum.setRotate(0.60f);
          } else {
            mecanum.setRotate(0.0f); 
            autoState = "salad"; 
          }
        break; 
        
        case "salad":
          hand.setPosition(0.3); 
          armState = "0block"; 
          moveArm();
          stayOnTarget(178,-178);
          if(frontRange.getDistance(DistanceUnit.CM) <= 80){
            mecanum.setFwd(0.70f); 
          } else {
            mecanum.setFwd(0.0f); 
            autoState = "deadBodies"; 
          }
          telemetry.addData("frontDist", frontRange.getDistance(DistanceUnit.CM));
        break; 

        case "deadBodies":
          hand.setPosition(0.3); 
          moveArm();
          leftFound.setPosition(0.4);
          rightFound.setPosition(0.5);
          if(leftFound.getPosition() == 0.4){
            autoState = "dragBody"; 
          }
        break;

        case "dragBody":
          moveArm();
          stayOnTarget(178, -178); 
          if(frontRange.getDistance(DistanceUnit.CM) >= 45){
            mecanum.setFwd(-0.8f);
          } else {
            mecanum.setFwd(0.0f);
            autoState = "spinFound"; 
          }
        break; 
        
        case "spinFound":
          moveArm();
          if(angles.firstAngle > 0 || angles.firstAngle <= -95){
            mecanum.setRotate(-0.65f);
          } else {
            mecanum.setRotate(0.0f);
            autoState = "releaseFound";  
          }
        break; 

        case "releaseFound":
          stayOnTarget(-88, -92); 
          leftFound.setPosition(1);
          rightFound.setPosition(0); 
          if(leftFound.getPosition() == 1){
            autoState = "slideToLeft";
          }
        break;

        case "slideToLeft":
          stayOnTarget(-88, -92); 
          if(rightDist.getDistance(DistanceUnit.CM) > 20){
            mecanum.setSlide(0.8f);
            mecanum.setFwd(-0.05f);
          } else {
            mecanum.setSlide(0.0f);
            mecanum.setFwd(0.0f);
            currentPos = mecanum.getFrontRight(); 
            autoState = "parkBot"; 
          }
        break;

        case "parkBot":
          stayOnTarget(-88, -92); 
          telemetry.addData("currentPos:", currentPos); 
          if(mecanum.getFrontRight() > currentPos - 1550){
            mecanum.setFwd(-0.8f);
          } else {
            mecanum.setFwd(0.0f); 
          }
        break;

        }  
    }

    private void stayOnTarget(int topAngle, int bottomAngle) {
      if(angles.firstAngle > topAngle){
          mecanum.setRotate(0.25f);
      } else if(angles.firstAngle < bottomAngle) {
          mecanum.setRotate(-0.25f); 
      } else if(topAngle > angles.firstAngle && angles.firstAngle > bottomAngle) {
          mecanum.setRotate(0.0f); 
      }
    }
}