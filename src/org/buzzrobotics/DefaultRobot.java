/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2010. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.buzzrobotics;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Jaguar;  
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;


/**
 * Autonomous Mode:
 * - 0 = Do Nothing
 * - 1 = Far Field Triple Kick (Encoders) - untested
 * - 2 = Mid Field Double Kick (Encoders) - untested
 * - 3 = Near Field Single Kick           
 * - 4 = Move Forward, Kick, Continue
 * - 5 = Far Field Triple Kick (Camera)   - untested
 *
 *
 * This code assumes the following connections:
 * - Driver Station:
 *  - USB 1 - Driver. Arcade Drive.    (Right Stick)
 *  - USB 2 - Elevator Operator.       (Left Stick)
 *  - USB 3 - Autonomous Box.
 *  - USB 4 - Stop Button.
 *  
 *
 * - Left Elevator Joystick Buttons:
 *   - 1) Activite tilt and lift (Hold)
 *   - 2)                                 7) Medium Tension
 *   - 3) Park Arm                        8) Low Tension
 *   - 4) On Possessor                    9) Unlock Servos
 *   - 5) Off Possessor                  10) Lock Servo
 *   - 6) High Tension                   11) 
 *
 *   - X-Axis) Tilt
 *   - Y-Axis) Lift
 *
 *
 * - Right Drive Joystick Buttons:
 *   - 1) Kick (and then right to stow)
 *   - 2)                                 7) 
 *   - 3) Camera Down/Up (Hold)           8)
 *   - 4)                                 9) Reset Encoders
 *   - 5)                                10) Display camera data to this computer
 *   - 6) Manual Shut Of Kicker (Hold)   11) Display data to this computer
 *   
 *
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1 + 2 - Left and Right Drive. (Jaguars)          - PWM 7 - Kicker Tensioner
 *     - PWM 3 - Elevator Lift                                - PWM 8 - Camera Servo
 *     - PWM 4 - Elevator Tilt                                - PWM 9 - Servo Lift Lock
 *     - PWM 5 - Kicker                                      - PWM 10 - Servo Tilt Lock
 *     - PWM 6 - Possessor
 *
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class DefaultRobot extends IterativeRobot {

/********************************** Global Variables *************************************/
//Global Variables

           //Joysticks
                final static int driverJoystick = 1;        //Right Stick.
                final static int elevatorJoystick = 2;      //Left Stick.


           //RIGHT Joystick Buttons
                final static int rightButton1 = 1;
                final static int rightButton2 = 2;
                final static int rightButton3 = 3;
                final static int rightButton4 = 4;
                final static int rightButton5 = 5;
                final static int rightButton6 = 6;
                final static int rightButton7 = 7;
                final static int rightButton8 = 8;
                final static int rightButton9 = 9;
                final static int rightButton10 = 10;
                final static int rightButton11 = 11;


           //LEFT Joystick Buttons
                final static int leftButton1 = 1;
                final static int leftButton2 = 2;
                final static int leftButton3 = 3;
                final static int leftButton4 = 4;
                final static int leftButton5 = 5;
                final static int leftButton6 = 6;
                final static int leftButton7 = 7;
                final static int leftButton8 = 8;
                final static int leftButton9 = 9;
                final static int leftButton10 = 10;
                final static int leftButton11 = 11;


           //PWM ports
                final static int liftJaguarPWM = 3;         //Lift Victor.
                final static int tiltVictorPWM = 4;         //Tilt Victor.
                final static int kickerVictorPWM = 5;       //Kicker Victor.
                final static int possessorVictorPWM = 6;    //Possessor Victor.
                final static int tensionerJaguar = 7;       //Tensioner Victor.
                final static int cameraServoPWM = 8;        //Camera Servo
                final static int liftServoPWM = 9;          //Lift Lock.
                final static int tiltServoPWM = 10;         //Tilt Lock.


           //Relay
                final static int possessorLightPWM = 1;     //Possessor Light Spike.
                final static int targetLightPWM = 2;        //Target Light Spike.


           //Analog ports
                final static int liftAnalog = 2;            //Lift Pot
                final static int tiltAnalog = 3;            //Tilt Pot
                final static int tensionerAnalog = 4;       //Tensioner Pot
                final static int kickerAnalogProxLatch2 = 5;//prox sensor #2-unarmed
                final static int kickerAnalogProxLatch1 = 6;//prox sensor #1-shoot armed
                final static int ballDetectAnalog = 7;      //Ball Detect IR sensor


           //Threshs, ballDetect, kicker latch 1+2
                final static double kickerLatchThresh1 = 0.4;   //Kicker Thresh. Armed. We found metal.
                final static double kickerLatchThresh2 = 1.0;   //Kicker Thresh. Unarmed. We found metal.
                final static double ballDetectThresh = 4.6;    //Ball Detech Thresh. (ir sensor)


           //Tension
                final static double highTension = 0.61;         //FAR TENSION Variable.
                final static double mediumTension = 2.5;        //MEDIUM TENSION Variable.
                final static double lowTension = 4.4;           //LOW TENSION Variable.
                int tensionButton = 0;                          //Stop tension from moving
                double tensionSetpoint = 0;


           //Tension Buttons
                final static int highTensionButton = 6;
                final static int mediumTensionButton = 7;
                final static int lowTensionButton = 8;


           //Encoders
                final static int encoderOneSourceA = 1;        //Encoder 1 Source A
                final static int encoderOneSourceB = 2;        //Encoder 1 Source B
                final static int encoderTwoSourceA = 3;        //Encoder 2 Source A
                final static int encoderTwoSourceB = 4;        //Encoder 2 Source A
                static int encoderCountsLeft;                  //Left Drive Encoder Counts
                static int encoderCountsRight;                 //Right Drive Encoder Counts


           //Camera
                static double majorRadius;
                static double cameraRange;
                static double cameraAngle;
                static double kScoreThreshold = .01;
                static double currentAngle;                     //current camera angle
                static double savedAngle;                       // to remember autonomous pre-turn angle
                static double m_majorRadius;


           //Watchdog
                final static double watchdogExpiration = 5;     //Watchdog expiration is 5 seconds


           //Autonomous
                double autoMode = 0;                            //Used to stop autonomous from running in teleop
                double autoDelay = 0;                           //Set the desired delay for each autonomous mode
                static double delta = 0;                        //autonomous code
                static double previousDelta = 0;                //autonomous code


           //Gyro
                double gyroAngle;       //Gyro angle
                double autoGyroAngle;   //gyro angle for autonomous only
                int foundTarget = 0;                            //??


           //LIFT POT limits
                static double potLiftUpperLimit = 2.58;     //Lift pot upper limit
                static double potLiftLowerLimit = 0.86;     //Lift pot lower limit

           //TILT POT limits
                static double potTiltLeftLimit = 2.65;      //Tilt pot Left limit
                static double potTiltRightLimit = 2.12;     //Tilt pot Right limit



/********************************** Declare Variables *************************************/
//Declare Variables

           //Drive
                RobotDrive robotDriveTrain;

           // Start compressor and Camera.
                AxisCamera robotCamera;

           //Joysticks
                Joystick rightStick;			// Joystick 1 (Arcade Drive)
                Joystick leftStick;			// Joystick 2 (Elevator)

           //Victors
                Victor robotVictorPossessor;
                Victor robotVictorKicker;

           //Jaguars
                Jaguar robotJaguarLift;
                Jaguar robotJaguarTilt;
                Jaguar robotJaguarTensioner;

                                    //BREAD BOARD CODE ONLY
                                    //Victor robotJaguarLift;
                                    //Victor robotVictorTilt;
                                    //Jaguar robotJaguarKicker;
                                    //Victor robotJaguarTensioner;

           //Relays
                Relay robotSpikePossessorLight;
                Relay robotSpikeTargetLight;

           //Servos
                Servo robotServoLift;
                Servo robotServoTilt;
                Servo robotServoCamera;

           //Analog
                AnalogChannel analogBallDetect;
                AnalogChannel analogRobotPotLift;          //Lift Pot
                AnalogChannel analogRobotPotTilt;          //Tilt Pot 2
                AnalogChannel analogPotTensioner;          //Tensioner Pot (Kicker Tentsioner)
                AnalogChannel kickerProx1Kick;             //Prox #1 - Armed
                AnalogChannel kickerProx2Tuck;             //Prox #2 - Unarmed

           //Encoders
                Encoder robotEncoderLeft;
                Encoder robotEncoderRight;

           //Gyro
                Gyro gyro;


        // Declare a variable to use to access the driver station object(DS update stuff)
        DriverStation m_ds;                      // driver station object
        int m_priorPacketNumber;                 // keep track of the most recent packet number from the DS
        int m_dsPacketsReceivedInCurrentSecond;	 // keep track of the ds packets received in the current second

        static final int NUM_JOYSTICK_BUTTONS = 16;
        boolean[] m_rightStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];
        boolean[] m_leftStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];

                

/********************************** Local Variables *************************************/
//Local Variables - INT/BOOLEAN (Some used to count the number of periodic loops performed)

           //INT
                int m_driveMode;
                int m_autoPeriodicLoops;
                int m_disabledPeriodicLoops;
                int m_telePeriodicLoops;
                int m_rawEncoderCounts;
                int m_ballKickCount;
                int m_possessorSeconds;
                int m_currentTime;

           //BOOLEAN
                boolean m_previousStatePossesser;
                boolean m_prevousStateServoLift;
                boolean m_prevousStateServoTilt;
                boolean m_prevousStateServoCamera;
                boolean m_servoLiftLatch;
                boolean m_servoTiltLatch;
                boolean m_servoCameraLatch;
                boolean proximityLatchKicker = true;
                boolean proximityLatchTuck = true;
                boolean m_VictorPossessorLatch ;
                boolean tensionHighLow = true;
                boolean newTensionHighLow = true;
                boolean possessedBall = false;
                boolean tucking = false;
                boolean arming = false;
                boolean autonomousCase5 = false;
                boolean done = false;            //if auto is done then stop the autonomous
                boolean leftButtonThreeWasPressed = false;   //if the right button 2 was pressed then stow


    /**
     * Constructor for this "DefaultRobot" Class.
     *
     * The constructor creates all of the objects used for the different inputs and outputs of
     * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
     * providing named objects for each of the robot interfaces.
     */


/********************************** DefaultRobot *************************************/
    public DefaultRobot() {
        System.out.println("DefaultRobot Constructor Started\n");


           //Arcade Drive.
                robotDriveTrain = new RobotDrive(1,2);

           //Joysticks
                rightStick = new Joystick(driverJoystick);
                leftStick = new Joystick(elevatorJoystick);

           //Victors
                robotVictorPossessor = new Victor(1,possessorVictorPWM);
                robotVictorPossessor.setBounds(255,235,128,120,0);
                robotVictorKicker = new Victor(1,kickerVictorPWM);
                robotVictorKicker.setBounds(255,235,128,120,0);          // setBounds may not be needed
//                m_robotVictor.set(1.0);

           //Jaguars
                robotJaguarLift = new Jaguar(1,liftJaguarPWM);
                robotJaguarTilt = new Jaguar(1,tiltVictorPWM);
                robotJaguarTensioner = new Jaguar(1,tensionerJaguar);

                                //BREAD BOARD CODE ONLY
                                //robotJaguarKicker = new Jaguar(4,kickerJaguarPWM);
                                //robotJaguarKicker.setBounds(255,235,128,120,0);          // setBounds may not be needed
                                //robotJaguarTensioner = new Victor(4,tensionerJaguar);

           //Servos
                robotServoLift = new Servo(1,liftServoPWM);
                robotServoTilt = new Servo(1,tiltServoPWM);
                robotServoCamera = new Servo(1,cameraServoPWM);

           //Relays
                robotSpikePossessorLight = new Relay(1,possessorLightPWM,Relay.Direction.kForward);
                robotSpikePossessorLight.set(Relay.Value.kOff);
                robotSpikeTargetLight = new Relay(1,targetLightPWM,Relay.Direction.kForward);
                robotSpikeTargetLight.set(Relay.Value.kOff);

           //Analogs---slot 6 on cRIO
                analogBallDetect = new AnalogChannel(1, ballDetectAnalog);
                analogBallDetect.setAverageBits(3);
                analogRobotPotLift = new AnalogChannel(1,liftAnalog);            //pot lift
                analogRobotPotTilt = new AnalogChannel(1,tiltAnalog);            //pot tilt
                analogPotTensioner = new AnalogChannel(1,tensionerAnalog);       //pot tensioner
                kickerProx1Kick = new AnalogChannel(1,kickerAnalogProxLatch1);   //prox #1-shoot armed
                kickerProx2Tuck = new AnalogChannel(1,kickerAnalogProxLatch2);   //prox #2-unarmed

           //Encoder
                robotEncoderLeft = new Encoder(encoderOneSourceA,encoderOneSourceB,false,EncodingType.k4X);
                robotEncoderLeft.start();
                robotEncoderLeft.reset();
                robotEncoderRight = new Encoder(encoderTwoSourceA,encoderTwoSourceB,false,EncodingType.k4X);
                robotEncoderRight.start();
                robotEncoderRight.reset();

           //Gyro
                gyro = new Gyro(1, 1);


           //Driver Station
                m_ds = DriverStation.getInstance();         //used in "autonomusReadings"
                m_priorPacketNumber = 0;
                m_dsPacketsReceivedInCurrentSecond = 0;

           //Other
                m_VictorPossessorLatch = false;
                m_previousStatePossesser = false;
                m_servoLiftLatch = false;
                m_servoCameraLatch = false;
                m_prevousStateServoLift = false;
                m_prevousStateServoCamera = false;


            // Iterate over all the buttons on each joystick, setting state to false for each
                int buttonNum = 1;			// start counting buttons at button 1
                    for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
                            m_rightStickButtonState[buttonNum] = false;
                            m_leftStickButtonState[buttonNum] = false;
                    }


            // Initialize counters to record the number of loops completed in autonomous and teleop modes
                m_autoPeriodicLoops = 0;
                m_disabledPeriodicLoops = 0;
                m_telePeriodicLoops = 0;


            //Print out to the computer "BuiltinDefaltCode Constructor Completed/n"
                System.out.println("DefaultRobot Constructor Completed\n");
            }


/********************************** Init Routines *************************************/
// Actions which would be performed once (and only once) upon initialization of the
// robot would be put here.

    public void robotInit() {
            
                System.out.println("RobotInit() completed.\n");
                Timer.delay(7.0);                                                 //Should this be shortened??
                robotCamera = AxisCamera.getInstance();
                robotCamera.writeResolution(AxisCamera.ResolutionT.k160x120);   //(640x480 is the highest, 160x120 is the lowest)
                robotCamera.writeBrightness(0);
                robotCamera.writeCompression(80);
                Watchdog.getInstance().setExpiration(watchdogExpiration);
                Watchdog.getInstance().feed();

           //Unlock both servos
                robotServoLift.setRaw(1);     //Unlock lift
                robotServoTilt.setRaw(255);   //Unlock tilt
                done = false;                 //We are not done with autonomous
                m_currentTime = 0;            //the current time that we have had ball is 0s
                m_possessorSeconds = 0;       //0 seconds for poss
                leftButtonThreeWasPressed = false;      //this explains itself....
    }

        
    public void disabledInit() {
                m_disabledPeriodicLoops = 0;	// Reset the loop counter for disabled mode
    }


    public void autonomousInit() {
                m_autoPeriodicLoops = 0;	    // Reset the loop counter for autonomous mode???
                Watchdog.getInstance().feed();      //Feed the watchdog
                autonomousCase5 = false;            //We are not in case 1 yet..
                autonomusReadings();                //Read the analog box voltage and determine which autonomous mode to use
                gyro.reset();                       //Reset the gyro before automonomus starts
                done = false;                       //We are not done with autonomous
                m_ballKickCount = 0;                //Used to keep count of how many balls have been kicked. (current = 0)
                leftButtonThreeWasPressed = false;

           //Unlock the servos before autonomous
                robotServoLift.setRaw(1);           //unlock lift
                robotServoTilt.setRaw(255);         //unlock tilt

           //Stop the Kicker
                proximityLatchKicker = true;

           //Turn off lights
                robotSpikePossessorLight.set(Relay.Value.kOff);      //Possessor light off
                robotSpikeTargetLight.set(Relay.Value.kOff);         //Turn the camera light off


           //At the begining of autonomous we arm the robot's kicker(if we aren't in the armed poss then move the kicker until we are)
                if (kickerProx1Kick.getVoltage() <= kickerLatchThresh1) {       //If we haven't found the metal then move
                    robotVictorKicker.setRaw(1);                                //move until we find the metal (the kick metal)
                    proximityLatchKicker = false;
                } else {                          //If we have found it then stop the kicker from runing
                    robotVictorKicker.set(0);
                    proximityLatchKicker = true;
                }
    }


    public void teleopInit() {
                Watchdog.getInstance().feed();                  // Feed the Watchdog
                m_telePeriodicLoops = 0;                        // Reset the loop counter for teleop mode
                m_dsPacketsReceivedInCurrentSecond = 0;         // Reset the number of dsPackets in current second
                gyro.reset();                                   //After autonomous reset the gyro
                leftButtonThreeWasPressed = false;              //Explains itself...


           // Move kicker from arm poss to stow poss(to set us up from teleop kicking)
                while(kickerProx2Tuck.getVoltage() < kickerLatchThresh2){
                    robotVictorKicker.setRaw(1);
                    Watchdog.getInstance().feed();  //Feed the watchdog
                }
                robotVictorKicker.set(0);   //we have now found the stow poss so we stop the kicker motor


           //Stop the kicker
                proximityLatchKicker = true;
                proximityLatchTuck = true;

           //We aren't hitting the tuck or arm button
                arming = false;
                tucking = false;

           //Unlock both servos before we hit teleop periodic
                robotServoLift.setRaw(1);     //unlock lift
                robotServoTilt.setRaw(255);   //unlock tilt

           //STOP EVERYTHING FROM AUTONOMOUS(after autonomous ends)
                robotVictorKicker.set(0);
                robotJaguarLift.set(0);
                robotJaguarTensioner.set(0);
                robotVictorPossessor.set(0);
                robotJaguarTilt.set(0);
                robotDriveTrain.arcadeDrive(0,0);
                robotSpikePossessorLight.set(Relay.Value.kOff);   //Possessor light off
                robotSpikeTargetLight.set(Relay.Value.kOff);    //Turn the camera light off

           //STOP AUTONOMOUS FROM RUNNING IN TELEOP
                autoMode = 0;   //return or "skip" the autonomous code
                
/*
              if (kickerProx2Tuck.getVoltage() <= kickerLatchThresh2) {
                 robotVictorKicker.setRaw(1);
                 tucking = false;
               } else {
                 robotVictorKicker.set(0);
                 tucking = true;
               }
*/          
    }


/********************************** Periodic Routines *************************************/
static int printSec = (int)((Timer.getUsClock() / 1000000.0) + 1.0);    //Just getting the time...in seconds
static final int startSec = (int)(Timer.getUsClock() / 1000000.0);

    public void disabledPeriodic()  {
            Watchdog.getInstance().feed();

            //Increment the number of disabled periodic loops completed.
            m_disabledPeriodicLoops++;  //???

            // While disabled, printout the duration of current disabled mode in seconds on this computer.
            if ((Timer.getUsClock() / 1000000.0) > printSec) {
                System.out.println("Disabled seconds: " + (printSec - startSec) + "\r\n");
                printSec++;
            }
    }



/********************************** Autonomous Periodic *************************************/
//Autonomous code
//BEGIN AUTONOMOUS MODE!!! Use the analog volatage number select the autonomous mode.
    public void autonomousPeriodic() {
        
        Watchdog.getInstance().feed();
        m_autoPeriodicLoops++;
                    
        if (autoMode == 0) return;     //If 0 then do nothing.

        //reset encoders to count distance
        robotEncoderLeft.reset();
        robotEncoderRight.reset();
        stopTension();      //When we come up to tension stop the jaguar.

        //Select our autonomous case
        switch ((int)autoMode) {      //The Autonomous Box Reading Determines what autonomous mode we are in

           case 1:     //Far Field Triple Kick with encoders(gyro commented out)
                    Timer.delay(0);     //zero delay
                    autoDelay = 0;
                    if (m_ballKickCount == 3) break;

                    if (analogBallDetect.getVoltage() < ballDetectThresh) {  //We found a ball
                            robotDriveTrain.arcadeDrive(0, 0);  //stop once we find a ball
                            //reset the gyro, left and right encoders while we aren't moving
//                                gyro.reset();
                            robotEncoderLeft.reset();
                            robotEncoderRight.reset();
                            robotSpikePossessorLight.set(Relay.Value.kOn);       //Possessor light on
                            Timer.delay(0.1);   //small delay to ensure that the gyro/elevators get reset

                            //get the encoder counts
                            robotDriveTrain.arcadeDrive(0,0.4); //turn left
                            encoderCountsLeft = robotEncoderLeft.get();
                            encoderCountsRight = robotEncoderRight.get();
                            while (encoderCountsRight <= -64) {  //we will continue to turn until we are at the correct angle
                                //get the encoder counts
                                Watchdog.getInstance().feed();
                                encoderCountsRight = robotEncoderRight.get();
                            }

                            //we now have hit the 64th encoder count on the right drive - stop and kick
                            //note: appears autonomous mode has 1/4 sensitivity of teleop mode
                            //      thus: 1 turn = 128 counts in autonomous, 1 turn = 512 counts in teleop
                            robotDriveTrain.arcadeDrive(0, 0);    //stop drive
                            robotVictorPossessor.set(0);          //stop the possessor
                            autonomusKick(true);                  //kick
                            Timer.delay(0.4);                     //small delay so the kicker can go all the way around
                            while(!proximityLatchKicker) {
                                Watchdog.getInstance().feed();
                                autonomusKick(false);             //stop when you see the metal
                            }

                            m_ballKickCount++;

                            robotDriveTrain.arcadeDrive(0.0, -0.4); //turn right(to go back to where you were)

                            while(encoderCountsRight > 0) {
                                Watchdog.getInstance().feed();
                                encoderCountsRight = robotEncoderRight.get();
                            }

                            robotDriveTrain.arcadeDrive(0,0);   //stop the drive




                        } else {    //drive to look for a ball
                            robotDriveTrain.arcadeDrive(-0.42, 0);                //Move forward at 50% speed until we find a ball
                            robotSpikePossessorLight.set(Relay.Value.kOff);      //Possessor light off
//                            robotSpikeTargetLight.set(Relay.Value.kOff);         //Turn the camera light off
                            robotVictorPossessor.set(1.0);                       //poss turns on
                            autonomusKick(false);
                        }
                     break;


           case 2:     //Mid Field Double Kick with encoders(gyro commented out)
                    Timer.delay(0);     //zero delay
                    autoDelay = 0;
                    if (m_ballKickCount == 2) break;

                    if (analogBallDetect.getVoltage() < ballDetectThresh) {  //We found a ball
                            robotDriveTrain.arcadeDrive(0, 0);  //stop once we find a ball
                            //reset the gyro, left and right encoders while we aren't moving
//                                gyro.reset();
                            robotEncoderLeft.reset();
                            robotEncoderRight.reset();
                            robotSpikePossessorLight.set(Relay.Value.kOn);       //Possessor light on
                            Timer.delay(0.1);   //small delay to ensure that the gyro/elevators get reset

                            //get the encoder counts
                            robotDriveTrain.arcadeDrive(0,-0.6); //turn left was (0,0.4
                            encoderCountsLeft = robotEncoderLeft.get();
                            encoderCountsRight = robotEncoderRight.get();
                            while (encoderCountsRight <= 96) {  //we will continue to turn until we are at the correct angle
                                //get the encoder counts
                                Watchdog.getInstance().feed();
                                encoderCountsRight = robotEncoderRight.get();
                            }

                            //we now have hit the 96th encoder count on the right drive - stop and kick
                            //note: appears autonomous mode has 1/4 sensitivity of teleop mode
                            //      thus: 1 turn = 128 counts in autonomous, 1 turn = 512 counts in teleop
                            robotDriveTrain.arcadeDrive(0, 0);    //stop drive
                            robotVictorPossessor.set(0);          //stop the possessor
                            autonomusKick(true);                  //kick
                            Timer.delay(0.4);                     //small delay so the kicker can go all the way around
                            while(!proximityLatchKicker) {
                                Watchdog.getInstance().feed();
                                autonomusKick(false);             //stop when you see the metal
                            }

                            m_ballKickCount++;

                            robotDriveTrain.arcadeDrive(0, 0.6); //turn right(to go back to where you were) was (0,-0.4)

                            while(encoderCountsRight > 0) {
                                Watchdog.getInstance().feed();
                                encoderCountsRight = robotEncoderRight.get();
                            }

                            robotDriveTrain.arcadeDrive(0,0);   //stop the drive




                        } else {    //drive to look for a ball
                            robotDriveTrain.arcadeDrive(-0.42, 0);                //Move forward at 50% speed until we find a ball
                            robotSpikePossessorLight.set(Relay.Value.kOff);      //Possessor light off
     //                       robotSpikeTargetLight.set(Relay.Value.kOff);         //Turn the camera light off
                            robotVictorPossessor.set(1.0);                       //poss turns on
                            autonomusKick(false);
                        }
                     break;



           case 3:     //Near Field single kick
                    if(done == true) break;

                    Timer.delay(0); //zero delay
                    autoDelay = 0;
                            //begin to drive to look for a ball
                            robotDriveTrain.arcadeDrive(-0.5, 0);                //Move forward at 50% speed until we find a ball
                            robotSpikePossessorLight.set(Relay.Value.kOff);      //Possessor light off
                            robotVictorPossessor.set(1.0);                       //poss turns on
                            autonomusKick(false);
                                
                        if (analogBallDetect.getVoltage() < ballDetectThresh) {  //We found a ball
                                robotDriveTrain.arcadeDrive(0, 0);  //stop once we find a ball
                                robotVictorPossessor.set(0);          //stop the possessor
                                robotSpikePossessorLight.set(Relay.Value.kOn);       //Possessor light on
                                autonomusKick(true);                  //kick
                                Timer.delay(0.4);                     //small delay so the kicker can go all the way around
                                while(!proximityLatchKicker) {
                                    autonomusKick(false);             //stop when you see the metal
                                }
                                robotDriveTrain.arcadeDrive(0,0);
                                done = true; //you kicked your only ball so stop autoMode
                        }
                        

                 break; 

           case 4:  //Move Forward, Kick, Continue
           //testing the farfield triple kick-(drive, find ball-kick, drive, find ball-kick)
                   //note: appears autonomous mode has 1/4 sensitivity of teleop mode
                   //      thus: 1 turn = 128 counts in autonomous, 1 turn = 512 counts in teleop
                   //      thus: 1 wheel turn = 2 feet = 1650counts in teleop

           
                    Timer.delay(0);     //zero delay
                    autoDelay = 0;
                    //Once we have kicked 3 times stop the robot
                    if (m_ballKickCount == 3) break;
                    
                    //Once we have kicked 2 times set tension to medium
                    if (m_ballKickCount == 2) {
                        startTension(mediumTensionButton, mediumTension);
                    }


                    if (analogBallDetect.getVoltage() < ballDetectThresh) {  //We found a ball
      
                            robotSpikePossessorLight.set(Relay.Value.kOn);       //Turn on the possessor light on
                            robotDriveTrain.arcadeDrive(0, 0);    //stop drive
                            robotVictorPossessor.set(0);          //stop the possessor
                            autonomusKick(true);                  //kick
                            Timer.delay(0.4);                     //small delay so the kicker can go all the way around
                            while(!proximityLatchKicker) {
                                Watchdog.getInstance().feed();
                                autonomusKick(false);             //stop when you see the metal
                            }
        
                            m_ballKickCount++;              //add one ball to the ball count. once we hit 3 balls we stop autonomous

       
                    } else {    //drive to look for a ball
                            
                        //get encoder counts when we are moveing
                            encoderCountsLeft = robotEncoderLeft.get();
                            encoderCountsRight = robotEncoderRight.get();
                        //if our left encoder counts are more than 2900 then stop the drive + shut everything off
                                if((encoderCountsLeft) >= 2900) {   
                                    robotDriveTrain.arcadeDrive(0, 0);               //stop drive
                                    robotSpikePossessorLight.set(Relay.Value.kOff);  //Possessor light off
                                    robotVictorPossessor.set(0);                     //Poss turns off
                                    autonomusKick(false);                            //do not kick
                                    break;                                           //
                                
                                } else {
                                    robotDriveTrain.arcadeDrive(-0.5, 0.07);         //(-0.42 = 42% forward = good)
                                    robotSpikePossessorLight.set(Relay.Value.kOff);  //Possessor light off
                                    robotVictorPossessor.setRaw(255);                //poss turns on
                                    autonomusKick(false);                            //do not kick while moveing
                                }
                       }


       break;  //end of case 4



           case 5:  //drive forward for 2900 left encoder counts then stop.
                //get encoder counts when we are moveing
                    encoderCountsLeft = robotEncoderLeft.get();
                    encoderCountsRight = robotEncoderRight.get();
                //if our left encoder counts are more than 2900 then stop the drive + shut everything off
                        if((encoderCountsLeft) >= 2900) {
                            robotDriveTrain.arcadeDrive(0, 0);               //Stop drive
                            robotSpikePossessorLight.set(Relay.Value.kOff);  //Possessor light off
                            robotVictorPossessor.set(0);                     //Keep possessor off
                            autonomusKick(false);                            //Do not kick while moveing
                            break;                                           //Once we hit 2900 then stop automode

                        } else {
                            robotDriveTrain.arcadeDrive(-0.7, 0);           //-0.7 = 70% forward = fast
                            robotSpikePossessorLight.set(Relay.Value.kOff);  //Possessor light off
                            robotVictorPossessor.set(0);                     //Keep possessor off
                            autonomusKick(false);                            //do not kick while moveing
                        }

                            
/*         case 5:     //Far Field Triple Kick, with Camera
                    autonomousCase5 = true;     //set-don't skip the camera code

                    Timer.delay(autoDelay);     //Delay by the # of seconds that "autoDelay" says
                    autoDelay = 0;              //Only delay once

                        if (analogBallDetect.getVoltage() < ballDetectThresh) {  //We found a ball
                                robotDriveTrain.arcadeDrive(0, 0);  //stop once we find a ball
                           //     gyro.reset();   //reset the gyro while we aren't moving
                                robotSpikePossessorLight.set(Relay.Value.kOn);       //Possessor light on
                                cameraTarget(7); //start the camera

                                if (foundTarget == 1) {   //if our camera found a target
                                    robotDriveTrain.arcadeDrive(0, 0);    //stop drive
                                    savedAngle = currentAngle;
                                    robotSpikeTargetLight.set(Relay.Value.kOn);    //Turn the camera light on
                                    robotVictorPossessor.set(1.0);      //poss on
                                    robotDriveTrain.arcadeDrive(0.0, 0.4); //left turn

                                    //our bot will continue to turn until the proper angle has been reached. we will then stop the drive
                                    while (java.lang.Math.abs(currentAngle) > java.lang.Math.abs(cameraAngle)) {
                                        cameraTarget(7);
                                    }

                                    robotDriveTrain.arcadeDrive(0, 0);  //stop drive
                                    robotVictorPossessor.set(0);      //stop the possessor
                                    autonomusKick(true);    //start kicker
                                    Timer.delay(0.4);       //small delay
                                    while(!proximityLatchKicker) {//until we have found the metal-spin
                                        autonomusKick(false);    //stop when you see the metal
                                    }
                                    robotDriveTrain.arcadeDrive(0.0, -0.4); //left right-return to where we were

                                    //delta-angle difference
                                    delta = (java.lang.Math.abs(currentAngle - savedAngle));
                                    previousDelta = delta;
                                    while(delta <= previousDelta) {
                                        cameraTarget(7);    //update the angles for the delta
                                        previousDelta = delta;  //place the new data in the variables
                                        delta = (java.lang.Math.abs(currentAngle - savedAngle));
                                    }

                                    robotDriveTrain.arcadeDrive(0,0);   //stop the bot before we continue looking for a ball
                                }

                                //using the gyro
                    //            gyro.reset();       //reset the gryo
                    //            autoGyroAngle = gyro.getAngle();    //we have found the ball-get the gyro angle of 0.
                                //robotDriveTrain.arcadeDrive(0.2,45); //turn to face the goal by 45degrees??


                                //turn on possessor until we find the target??
                                  //if the camera found the target shoot at it (with a delay to finish turning)


                            } else {
                                robotDriveTrain.arcadeDrive(-0.5, 0);                //Move forward at 50% speed until we find a ball
                                robotSpikePossessorLight.set(Relay.Value.kOff);      //Possessor light off
                                robotSpikeTargetLight.set(Relay.Value.kOff);         //Turn the camera light off
                                robotVictorPossessor.set(1.0);  //poss on
                                autonomusKick(false);
                            }

                            break;      //End of Triple Kick, with Camera
*/

                default:  {  //do nothing case
                    //stop everything from automode
                    robotVictorKicker.set(0);
                    autonomusKick(false);                            //do not kick instead of setting the kicker victor to 0
                    robotJaguarLift.set(0);
                    robotJaguarTensioner.set(0);
                    robotVictorPossessor.set(0);                    //Stop the possessor from moveing after we are done with autonomous
                    robotJaguarTilt.set(0);
                    robotDriveTrain.arcadeDrive(0,0);               //stop the bot before we continue looking for a ball
                    robotSpikePossessorLight.set(Relay.Value.kOff); //Possessor light off
                    return;     //then skip autonomous (do we need break?)
                }
                
            }  //End select of auto modes



                    /* the below code (if uncommented) would drive the robot forward at half speed
                     * for two seconds.  This code is provided as an example of how to drive the
                     * robot in autonomous mode, but is not enabled in the default code in order
                     * to prevent an unsuspecting team from having their robot drive autonomously!
                     */
                    /* below code commented out for safety
                    if (m_autoPeriodicLoops == 1) {
                            // When on the first periodic loop in autonomous mode, start driving forwards at half speed
                            m_robotDrive->Drive(0.5, 0.0);			// drive forwards at half speed
                    }
                    if (m_autoPeriodicLoops == (2 * GetLoopsPerSec())) {
                            // After 2 seconds, stop the robot
                            m_robotDrive->Drive(0.0, 0.0);			// stop robot
                    }
                    */
	}

/********************************** Teleop Periodic *************************************/
//Teleop code
    public void teleopPeriodic() {
                Watchdog.getInstance().feed();
                // Increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
                m_dsPacketsReceivedInCurrentSecond++;	// increment DS packets received
                //Stop autonomous from running in teleop
                autoMode = 0;
//                rightButtonTwoWasPressed = false;
                
                //If the beam is broken we have a ball --> turn on light
                    possession();

                //Call the camera method  to begin running the camera code
                    cameraTarget(7);

         
                //ARCADE DRIVE - DRIVE.
                    robotDriveTrain.arcadeDrive(rightStick,false);


                //KICKER - DRIVER - BUTTON 1.
                    kickArmed();    //When button 1 is pressed, button 3 code is disabled. ("arming kicker")
                    kickUnarmed();  //When button 3 is pressed, button 1 code is disabled. (kick/stow)

                    
                //ELEVATOR LIFT AND TILT - ELEVATOR - BUTTON 1 - HOLD
                    //when button 1 is presses, move the elevator up by 3"
                    if (this.getLeftButtonState(leftButton1)) {

                        double lift = analogRobotPotLift.getVoltage();
                        double tilt = analogRobotPotTilt.getVoltage();


                        //Elevator Lift
                        double y = (leftStick.getY());
                                //up down/ lift
                                if((y > 0.15) && (lift < potLiftUpperLimit)) { //upper limit of 2.935
                                    robotServoLift.setRaw(1);                 //unlock the lift servo
                                    robotJaguarLift.set(y);                   //Elevator. (Lift upwards)
                                }

                               //Lift downward
                               else if((y < -0.15) && (lift > potLiftLowerLimit)) {   //lower limit of 1.19
                                    robotServoLift.setRaw(1);                   //unlock the lift servo
                                    robotJaguarLift.set(y);                      //Elevator. (Lift downwards)
                                } else {
                                    robotServoLift.setRaw(1);                 //unlock the lift servo
                                    robotJaguarLift.set(0);                   //Don't move the elevator)
                                }


                        //Tilt
                        double x = (leftStick.getX());       
                             if((x > 0.15) && (tilt > potTiltRightLimit)) {     //right limit of 2.12
                                if(lift < 1.16) {                //move elevator up by 3"
                                    robotServoLift.setRaw(1);   //unlock the lift servo to move up
                                    robotJaguarLift.set(0.4);   //move lift up
                                    robotJaguarTilt.set(0);     //stop us from tilting until we are up those 3"
                                } else {
                                    robotServoTilt.setRaw(255);    //unlock the tilt servo
                                    robotJaguarTilt.set(x/2);    //Elevator. (Tilt right)
                                }
                             }

                                else if((x < -0.15) && (tilt < potTiltLeftLimit)) {   //left limit of 2.65
                                    if(lift > 2.33) {                 //where to move elevator down to when we move left(lift pot)
                                        robotServoLift.setRaw(1);    //unlock the tilt servo
                                        robotJaguarLift.set(-0.4);   //move lift down
                                        robotJaguarTilt.set(0);      //when we move left
                                    } else {
                                        robotServoTilt.setRaw(255);  //unlock the tilt servo
                                        robotJaguarTilt.set(x/2);    //Elevator. (Tilt left)
                                    }

                                } else {
                                    robotServoTilt.setRaw(255);      //unlock the tilt servo
                                    robotJaguarTilt.set(0);          //Don't move the elevator)
                                }


                             
                    } else {   
                           robotJaguarLift.set(0);          //Don't move the elevator)
                           robotJaguarTilt.set(0);          //Don't move the elevator)
                        
                    }



                //PARK BUTTON - ELEVATOR - BUTTON 3
                if ((this.getLeftButtonState(leftButton3)) || leftButtonThreeWasPressed) {

                    leftButtonThreeWasPressed = true;    //we just pressed the stow button

                    //Read the lift and tilt pots
                    double lift = analogRobotPotLift.getVoltage();
//                    double tilt = analogRobotPotTilt.getVoltage();

                  //Lock both servos(we don't want the tilt to move + as we move down on lift it will ratchet to lock us on the last position)
                    robotServoLift.setRaw(255);     //lock lift
                    robotServoTilt.setRaw(1);       //lock tilt

                  //If lift is't down to park poss(lower limit on lift pot)
                        if(lift > potLiftLowerLimit) {       //lower limit of 1.12
                          robotJaguarLift.set(-1.0);         //move us until the lift is all the way down
                        } else {
                          robotJaguarLift.set(0);            //if we are at the lower limit (lift) then stop the jaguar
                          leftButtonThreeWasPressed = false;  //stop the loop from running the stow code
                        }                                    //from moveing the lift


                           //lock the servos then raise the robot by lowering the arm
         /*                   double lift = analogRobotPotLift.getVoltage();
                            double tilt = analogRobotPotTilt.getVoltage();
                            if(lift >= 2.1) {  //if lift is't down to park poss
                                robotJaguarLift.set(-0.1);   //move us until the lift is all the way down
                            } else {
                                robotJaguarLift.set(0);
                            }


                            if(tilt >= 2.08) {       //move to the ___ if we are to far over
                                robotVictorTilt.set(0.3);
                            } 
                            else if(tilt <= 2.00) {       //move to the ___ if we are to far over
                                robotVictorTilt.set(-0.3);
                            }

                            else {
                                robotVictorTilt.set(0);
                            }
          */



                } else {
                }


                //CAMERA SERVO - DRIVER - BUTTON 3 - HOLD
                    if (this.getRightButtonState(rightButton3) ^ m_prevousStateServoCamera) {
                         m_servoCameraLatch   = ! m_servoCameraLatch ;
                    }

                    m_prevousStateServoCamera = this.getRightButtonState(rightButton3);

                    if(m_servoCameraLatch){
                        robotServoCamera.setRaw(185); //down camera
                    } else {
                        robotServoCamera.setRaw(255); //up camera
                    }


                //ON POSSESSOR - ELEVATOR - BUTTON 4.
                    if (this.getLeftButtonState(leftButton4)) {
                        robotVictorPossessor.setRaw(255);         //1 or 255?? 
                    }
       
        /*            if (this.getLeftButtonState(leftButton4)) {
                        robotVictorPossessor.setRaw(145);
                    }
        */
                    else {
                    }


                //OFF POSSESSOR - ELEVATOR - BUTTON 5.
                    if (this.getLeftButtonState(leftButton5)) {
                        robotVictorPossessor.set(0.0);
                    }
        
         /*           if (this.getLeftButtonState(leftButton5)) {
                        robotVictorPossessor.set(0.0);
                    }
         */
                    else {
                    }


                //KICKER TENSIONER - ELEVATOR - FAR RANGE - MEDIUM RANGE - CLOSE RANGE
                //                              BUTTON 6    BUTTON 7       BUTTON 8
                
                     if (this.getLeftButtonState(highTensionButton)) {         //High = 6 button, elevator
                           startTension(highTensionButton,highTension);
                     }
                     if (this.getLeftButtonState(mediumTensionButton)) {       //Meduim = 7 button, elevator
                           startTension(mediumTensionButton, mediumTension);
                     }
                     if (this.getLeftButtonState(lowTensionButton))  {         //Low = 8 button, elevator
                              startTension(lowTensionButton,lowTension);
                     }

                     stopTension();     //When we come up to tension stop the jaguar.

/*
                //LIFT SERVO - ELEVATOR - BUTTON 9 - LATCHED
                    if ((this.getLeftButtonState(leftButton9) ^ m_prevousStateServoLift)  & this.getLeftButtonState(leftButton9)){
                         m_servoLiftLatch   = ! m_servoLiftLatch  ;
                    }

                    m_prevousStateServoLift = this.getLeftButtonState(leftButton9);

                    if(m_servoLiftLatch){
                        robotServoLift.setRaw(1);
                    } else {
                        robotServoLift.setRaw(255);
                    }

     
                //TILT SERVO - ELEVATOR - BUTTON 10 - LATCHED
                    if ((this.getLeftButtonState(leftButton10) ^ m_prevousStateServoTilt)  & this.getLeftButtonState(leftButton10)){
                         m_servoTiltLatch   = ! m_servoTiltLatch  ;
                    }

                    m_prevousStateServoTilt = this.getLeftButtonState(leftButton10);

                    if(m_servoTiltLatch){
                        robotServoTilt.setRaw(255);

                    } else {
                        robotServoTilt.setRaw(1);
                    }
*/

                 //UNLOCK BOTH SERVOS - ELEVATOR - BUTTON 9
                     if (this.getLeftButtonState(leftButton9)) {   //button 9, elevator, unlock both servos
                        robotServoLift.setRaw(1);   //unlock lift
                        robotServoTilt.setRaw(255);   //unlock tilt
                     } else {   //do nothing (hit button 10 to lock them)
                     }

                 //LOCK BOTH SERVOS - ELEVATOR - BUTTON 10
                     if (this.getLeftButtonState(leftButton10)) {   //button 10, elevator, lock both servos
                        robotServoLift.setRaw(255);   //lock lift
                        robotServoTilt.setRaw(1);   //lock tilt
                     } else {   //do nothing (hit button 9 to unlock them)
                     }

                  //MANUAL STOP THE KICKER - DRIVER - BUTTON 6
                     if (this.getRightButtonState(rightButton6)) {   //button 6 drive, manual kicker stop
                        robotVictorKicker.set(0);   //stop the kicker
                     } else {   
                     }


/*
                //CAMERA TRCAK - DRIVER - BUTTON 7
                    if(this.getRightButtonState(rightButton2)) {
                        cameraTarget(7);
                    }
                    else {
                        robotSpikeTargetLight.set(Relay.Value.kOff);
                    }
*/


/*
   //USED TO LATCH A VICTOR
   //Button 4: Possesser latch. Button 4 left stick.
    if ((this.getLeftButtonState(4) ^ m_previousStatePossesser) & this.getLeftButtonState(4)){
        m_VictorPossessorLatch   = ! m_VictorPossessorLatch  ;
    }
    m_previousStatePossesser = this.getLeftButtonState(4);

    if (m_VictorPossessorLatch) {
        robotVictorPossesser.set(1.0);
    }
    else {
        robotVictorPossesser.set(0.0);
    }
*/


           //FOR TESTING ONLY

                //RESET ENCODER ONE + TWO - RIGHT - BUTTON 9
                    if(this.getRightButtonState(rightButton9)) robotEncoderLeft.reset();
                    if(this.getRightButtonState(rightButton9)) robotEncoderRight.reset();
//                    if(this.getRightButtonState(rightButton10)) gyro.reset();

                //DATA PRINT TO THIS COMPUTER - RIGHT - BUTTON 11
                    if(this.getRightButtonState(rightButton11)) {
                        System.out.println("Encoder left: " + robotEncoderLeft.getRaw());         //Print encoderOne value
                        System.out.println("Encoder right: " + robotEncoderRight.getRaw());         //Print encoderTwo value
                        System.out.println("Ball Detect: " + possessedBall);                    //Print ball detect value
                        System.out.println("Ball Detect voltage: " + analogBallDetect.getVoltage());  //analog voltage
                        System.out.println("Pot lift: " + analogRobotPotLift.getVoltage());        //Print the lift pot value
                        System.out.println("Pot tilt: " + analogRobotPotTilt.getVoltage());        //Print the tilt pot value
                        System.out.println("Pot tensioner: " + analogPotTensioner.getVoltage());   //Print the tensioner pot value
                        System.out.println("Proximity Kicker: " + kickerProx1Kick.getVoltage());           //Print the proximity value #1
                        System.out.println("Proximity Tuck: " + kickerProx2Tuck.getVoltage());           //Print the proximity value #2
                        System.out.println("Auto Mode " + autoMode);           //Print the analog value of the autoMode

                    }
        }


/********************************** Autonomous Box *************************************/
//Autonomous box decides what autonomous mode we select.
    public void autonomusReadings() {
            autoMode = m_ds.getAnalogIn(1);

            autoMode = (int)((autoMode/135.0)+0.5);        //set autoMode to the autonomus mode.
            switch ((int)autoMode) {

                case 1:
                    System.out.println("Auto Mode 1   Far Field Triple kick ");
                    startTension(highTensionButton,highTension);
                    autoDelay = 0;
                    break;
                case 2:
                    System.out.println("Auto Mode 2   Mid Field Double kick");
                    startTension(mediumTensionButton,mediumTension);
                    autoDelay = 0;
                    break;
                case 3:
                    System.out.println("Auto Mode 3   Near Field Single Kick");
                    startTension(lowTensionButton,lowTension);
                    autoDelay = 0;
                    break;
                case 4:
                    System.out.println("Auto Mode 4   Move forward, kick, repeat");
                    startTension(highTensionButton,highTension);
                    autoDelay = 0;
                    break;
                case 5:
                    System.out.println("Auto Mode 5   Available!");
                    startTension(lowTensionButton,lowTension);
                    autoDelay = 1;
                    break;
                default:
                    System.out.println("Auto Mode 0   Do nothing");          //case 0
              
            }
        }


/********************************** Stop Tension *************************************/
//When we come up to tension stop the jaguar
    public void stopTension() {     
            if (tensionButton != 0) {     //if a button is hit do
                if (tensionHighLow != (analogPotTensioner.getVoltage() > tensionSetpoint)) {
                    tensionButton = 0;    //Stop because we have reached our goal
                    robotJaguarTensioner.set(0);
                }
            }
    }



/********************************** Autonomous Kick *************************************/
//Kick used in autonomus
    public void autonomusKick(boolean kick) {
            if(kickerProx1Kick.getVoltage() > kickerLatchThresh1) {      //kicker thresh = 1--(we found the metal)
                proximityLatchKicker = true;       //if true we don't kick
            }

            if (kick) {
                proximityLatchKicker = false;      //if false we do kick
            }
            
            if (proximityLatchKicker) {            //True = stop the kicker motor
                robotVictorKicker.set(0.0);
            } else {                               //False = run the kicker motor
                robotVictorKicker.setRaw(1);
            }
        }



/********************************** Kick *************************************/
//If you hit the kick button then kick and don't let you hit the tuck button
//In Teleop we "aren't useing" kickArmed because we are stowing and kicking at the same time
//HOWEVER we are using kickArmed in Autonomous mode to arm the robot
    public void kickArmed() {

            if (tucking) return;   //when button 3 is pressed skip this code
                if (kickerProx1Kick.getVoltage() > kickerLatchThresh1) {  //kicker thresh = 1--(we found the metal)
                    proximityLatchKicker = true;
                }

/*                 if (this.getRightButtonState(rightButton1)) {
                    proximityLatchKicker = false;
                }
*/
//                if (this.getRightButtonState(rightButton1)) {   //kick on driver, button 1
//                    proximityLatchKicker = false;
//                }

                if (proximityLatchKicker) {
                    robotVictorKicker.set(0.0);
                    arming = false;
                } else {
                    robotVictorKicker.setRaw(1);    //can't accept a negative number, backwards full speed
                    arming = true;
                    //robotVictorKicker.setRaw(125);  //bread board
                }
    }   //End of kickArmed



/********************************** Stow (Tuck) *************************************/    
//Stow the kicker in Autonomous mode
//In Teleop we use kickUnarmed as both the stow and as kick
//as we hit the kick button (to run kickUnarmed) we kick then head right back to stow
    public void kickUnarmed() {

          if (arming) return;  //when kick is pressed skip this code.
//            if (this.getRightButtonState(rightButton3)) {   //stow on driver
//                        proximityLatchTuck = false;
//                    }
                 if (kickerProx2Tuck.getVoltage() > kickerLatchThresh2) {    //kicker thresh = 1 (we found the metal)
                    proximityLatchTuck = true;
                 }

                 if (this.getRightButtonState(rightButton1)) {     //kick/stow on driver, button 1
                    proximityLatchTuck = false;
                 }
                                     
                     if (proximityLatchTuck) {     //is true stop the kicker
                         robotVictorKicker.set(0);
                         tucking = false;
                     } else {
                         robotVictorKicker.setRaw(1);   //can't accept a negative number, backwards full speed(we kick as we stow)
                         tucking = true;
                         //robotVictorKicker.setRaw(125); //bread board
                     }
            
    }   //End of kickUnarmed



/********************************** Start Tension *************************************/
//Depending what button is hit brings our tension up or down.
    public void startTension(int button, double tension) {
               tensionButton = button;
               tensionSetpoint = tension;

          if (analogPotTensioner.getVoltage() == tensionSetpoint) {
                   tensionButton = 0;
              }

          else {
              tensionHighLow = (analogPotTensioner.getVoltage() > tensionSetpoint); //is this true or false? false = drive until true

              if (tensionHighLow) {
                  robotJaguarTensioner.set(-0.5);
              }
              else {
                   robotJaguarTensioner.set(0.5);
              }
               }
    }



/********************************** Possession *************************************/
//When we have a ball turn the possessor light on. (+ run possessor for 2 seconds after ball detect is broken)
    public void possession() {  
            if (analogBallDetect.getVoltage() < ballDetectThresh) {  //If the beam is broken we have a ball.
                robotSpikePossessorLight.set(Relay.Value.kOn);       //Turn on the possessor light
                possessedBall = true;                               //ball


/*      //run the possessor for 2 seconds after the ball detect is broken
                    if (m_possessorSeconds == 0) {
                    m_possessorSeconds = (int)(Timer.getUsClock() / 1000000.0);
                    robotVictorPossessor.setRaw(255);   //on possessor(we found ball)
                    }

                    else {
                        m_currentTime = (int)(Timer.getUsClock() / 1000000.0);  //current time
                        if ((m_currentTime -  m_possessorSeconds) > 2)  {   //hold ball for two seconds then(compareing the real time)
                            robotVictorPossessor.set(0);    //poss off
                        }
                    }
*/
            } else {
                robotSpikePossessorLight.set(Relay.Value.kOff);      //Turn off the possessor light
                possessedBall = false;                              //no ball
  //              m_possessorSeconds = 0;                    //now 0 so we won't do anything
            }
    }



/********************************** Camera *************************************/
//Camera code
    //Camera code
    public void cameraTarget(int channelnum){
            Watchdog.getInstance().feed();

        try {
            if (robotCamera.freshImage()) {
            ColorImage image = robotCamera.getImage();
            Thread.yield();
            Target[] targets = Target.findCircularTargets(image);
            Thread.yield();
            image.free();

            if (targets.length == 0 || targets[0].m_score < kScoreThreshold) {
                robotSpikeTargetLight.set(Relay.Value.kOff);

                foundTarget = 0;        //for auto-we haven't found a target so don't shoot
                }

            else {  //we found the target. lights on.
                Watchdog.getInstance().feed();
                foundTarget = 1;    //for auto-we found a target shoot

                //Print camera data to this PC when button 10 is pressed.
                if(this.getRightButtonState(rightButton10)) System.out.println(targets[0]);
                if(this.getRightButtonState(rightButton10)) System.out.println("Target Angle: " + targets[0].getHorizontalAngle());

                //The farther we are the smaller the cameraAngle will be
                  majorRadius = targets[0].m_majorRadius;       //get majorRadius and us it to calculate the cameraRange then the cameraAngle
                    cameraRange = ((-232)*(majorRadius) + (55.45));
                    cameraAngle = ((-0.167)*(cameraRange) + (9.5));

                //compare to figure out if we should turn the light on or off.
                currentAngle = targets[0].getHorizontalAngle();
                if (java.lang.Math.abs(currentAngle) < java.lang.Math.abs(cameraAngle)) {
       //                     && (possessedBall)) {

                    robotSpikeTargetLight.set(Relay.Value.kOn);
                }
                else {
                    Watchdog.getInstance().feed();
                    robotSpikeTargetLight.set(Relay.Value.kOff);
                }

                }
            }
                
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        } catch (AxisCameraException ex) {
            ex.printStackTrace();
        }
    }
 



//????
    int GetLoopsPerSec() {
            return 10000;
    }


    private boolean  getRightButtonState(int  buttonNo){
        return rightStick.getRawButton(buttonNo);
    }


    private boolean  getLeftButtonState(int  buttonNo){
        return leftStick.getRawButton(buttonNo);
    }
}