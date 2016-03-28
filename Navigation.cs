using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;

        // The update speed of the sensor (in ms)
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 2;
        private double Kalpha = 8;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        //For Simulation Only
        //        const double betaTrackingAccuracy = 0.03;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;          //1000
        public double K_wheelRandomness = 10;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 10;         //3

        // Refresh rate of the laser (ms)
        private double laserRefresh = 20;

        private double stdDevGuess = 300;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }


        // Global Variables added
        private double psi1 = 0;
        private double psi2 = 0;
        public double K_Windup = 0.001;
        private double psi1_est = 0;
        private double psi2_est = 0;

        private double timeElapsedL = 0;
        private double timeElapsedR = 0;

        // For trajectory Tracking
        private int trajectoryState = 0;
        private bool isTrajectoryTracking = false;

        //For check dist function
        private bool inRange = false;
        private double phoTrajectoryAccuracy = .4;
        private double betaTrajectoryAccuracy = 1;

        // number of states
        private static int numTrajectoryStates = 7;


        private double[,] trajectoryArray = new double[7, 3] { { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 } };
        private double[,] trajLine = new double[7, 3] { { 1, 0, 0 }, { 2, 0, 0 }, { 3, 0, 0 }, { 4, 0, 0 }, { 5, 0, 0 }, { 5, 0, 0 }, { 5, 0, 0 } };
        private double[,] trajCircle = new double[7, 3] { { -2, 0, -2 }, { -1, -1.73, -0.78 }, { 1, -1.73, 0 }, { 2, 0, -0.78 }, { 2, 1, 1.7 }, { 1, 1.7, 2.7 }, { 0, 0, 0 } };
        //private double[,] trajCircle = new double[7, 3] { { 1, 1, 1.7 }, { 1, 1.7, 2.4 }, { -1, 1.7, 3 }, { 2, 0, -2.6 }, { -1, -1.7, -0.8 }, { 1, -1.73, 0 }, { 2, 0, 0.78 }, };

        //private double[,] trajCircle = new double[7, 3] { { 1, 1, 1.6 }, { 1, -1, 3 }, { -2, 0, -2 }, { 0, -2, 0}, { -1, -1.73, -0.78 }, { 1, -1.73, 0 }, { 2, 0, -0.78 }, };

        // 0 is use default, 1 is line, 2 is circle
        private int trajectorySelect = 2;

        //        private double feedForwardValue = 1;
        private double feedForwardValue = 4;


        // Rotation scaling

        // For hardware
        //private double k_rot = 6;

        //For feed forward
        private double k_rot = 20;
        // For Simulation
        //        private double k_rot = 60;


        

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            //Added by Ben Teng and Da Eun Shim
            if (isTrajectoryTracking)
            {
                initTrajectory();
            }
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    if (isTrajectoryTracking)
                    {
                        TrackTrajectory();
                    }

                    // Test wallDistance
                    //Console.WriteLine("Get Closest : {0}\r\n",map.GetClosestWallDistance(x_est, y_est, t_est));
//                    Console.WriteLine("Get Closest : {0}\r\n", map.GetClosestWallDistance(x_est, y_est, t_est));

                    double xVal = desiredX;
                    double yVal = desiredY;
                    double tVal = desiredT;

                    Console.WriteLine("Get Closest : {0}", map.GetClosestWallDistance(xVal, yVal, tVal));
                    Console.WriteLine("GetWall 0 {0} ", map.GetWallDistance(xVal, yVal, tVal, 0));
                    Console.WriteLine("GetWall 1 {0} ", map.GetWallDistance(xVal, yVal, tVal, 1));
                    Console.WriteLine("GetWall 2 {0} ", map.GetWallDistance(xVal, yVal, tVal, 2));
                    Console.WriteLine("GetWall 3 {0} ", map.GetWallDistance(xVal, yVal, tVal, 3));
                    Console.WriteLine("GetWall 4 {0} ", map.GetWallDistance(xVal, yVal, tVal, 4));
                    Console.WriteLine("GetWall 5 {0} ", map.GetWallDistance(xVal, yVal, tVal, 5));
                    Console.WriteLine("GetWall 6 {0} ", map.GetWallDistance(xVal, yVal, tVal, 6));
                    Console.WriteLine("GetWall 7 {0} ", map.GetWallDistance(xVal, yVal, tVal, 7));

 /*
                    Console.WriteLine("GetWall 1 {0} ", map.GetWallDistance(x_est, y_est, t_est, 1));
                    Console.WriteLine("GetWall 2 {0} ", map.GetWallDistance(x_est, y_est, t_est, 2));
                    Console.WriteLine("GetWall 3 {0} ", map.GetWallDistance(x_est, y_est, t_est, 3));
                    Console.WriteLine("GetWall 4 {0} ", map.GetWallDistance(x_est, y_est, t_est, 4));
                    Console.WriteLine("GetWall 5 {0} ", map.GetWallDistance(x_est, y_est, t_est, 5));
                    Console.WriteLine("GetWall 6 {0} ", map.GetWallDistance(x_est, y_est, t_est, 6));
                    Console.WriteLine("GetWall 7 {0} ", map.GetWallDistance(x_est, y_est, t_est, 7));
*/
                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= laserRefresh)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds.
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // ****************** Additional Student Code: Start ************

            // Students must set motorSignalL and motorSignalR. Make sure
            // they are set between 0 and maxPosOutput. A PID control is
            // suggested.



            // Gets current time elapsed
            TimeSpan ts = DateTime.Now - startTime;
            //Gets time step
            double timeStep = ts.TotalSeconds - time;
            //Updates current time
            time = ts.TotalSeconds;

            if (diffEncoderPulseL != 0)
            {
                // Warning? Worry about rollover
                double prevTime = timeElapsedL;
                timeElapsedL = time;

                psi1_est = diffEncoderPulseL / (timeElapsedL - prevTime);
            }
            if (diffEncoderPulseR != 0)
            {
                // Warning? Worry about rollover
                double prevTime = timeElapsedR;
                timeElapsedR = time;

                psi2_est = diffEncoderPulseR / (timeElapsedR - prevTime);
            }


            //Console.WriteLine("diffEncoderPulseL: {0} psi1_est: {1} timeStep: {2}\n", diffEncoderPulseL, psi1_est, timeStep);

            double prev_e_L = e_L;
            double prev_e_R = e_R;

            e_L = desiredRotRateL - psi1_est;
            e_R = desiredRotRateL - psi2_est;

            /*

            //Console.WriteLine("desiredRotRateL: {0} psi1_est: {1}\n", desiredRotRateL, psi1_est);

            // Sums all the previous error together and scales it by the estimated wheel velocity.
            // This attempts to remove windup
            e_sum_L = (e_sum_L + e_L) * K_Windup * Math.Abs(psi1_est);
            e_sum_R = (e_sum_R + e_R) * K_Windup * Math.Abs(psi2_est);

            //u_L = K_P * e_L + K_I * e_sum_L + K_D * (e_L - prev_e_L) / timeStep;
            //u_R = K_P * e_R + K_I * e_sum_R + K_D * (e_R - prev_e_R) / timeStep;

            u_L = K_P * e_L + desiredRotRateR;
            u_R = K_P * e_R + desiredRotRateR;

            //u_L = capControlSig(u_L);
            //u_R = capControlSig(u_R);

            capControlSig2();
            
            motorSignalL = (short)(zeroOutput + u_L);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);//(zeroOutput - u_R);

            Console.WriteLine("u_L: {0} u_R: {1}", u_L, u_R);
            Console.WriteLine("e_L: {0} e_R: {1}", e_L, e_R);
            Console.WriteLine("motorSignalL: {0} motorSignalR: {1}\n", motorSignalL, motorSignalR);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            */

            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.



            motorSignalL = (short)(zeroOutput + desiredRotRateL * 100 / feedForwardValue);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - desiredRotRateR * 100 / feedForwardValue);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));



            // ****************** Additional Student Code: End   ************

        }


        // Basic capping function
        double capControlSig(double u)
        {
            short zeroOutput = 16383;
            if (u > zeroOutput)
            {
                u = zeroOutput;
            }
            else if (u < -zeroOutput)
            {
                u = -zeroOutput;
            }
            return u;
        }

        // Capping function that scales the values relative to each other
        void capControlSig2()
        {
            // The maximum wheel rotation (rad/s) to get 0.25 m/s on the wheel
            double limit = 16383;
            if (Math.Abs(u_L) > limit || Math.Abs(u_R) > limit)
            {
                double maxScaling = Math.Max(Math.Abs(u_L), Math.Abs(u_R));
                u_L = u_L / maxScaling * limit;
                u_R = u_R / maxScaling * limit;
            }
        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];



            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

            double threshhold = 50;
            short speed = 9;
            double diff = Math.Abs(centralLaserRange - 1000);
            double gain = threshhold / speed;

            if (centralLaserRange < (1000 + threshhold) && centralLaserRange > (1000 - threshhold))
            {
                motorSignalR = 0; //desiredWheelSpeedL
                motorSignalL = 0; //desiredWheelSpeedL
            }

            else if (centralLaserRange > 1000 + threshhold)
            {
                motorSignalR = (short)(diff / gain); //desiredWheelSpeedL pulse/revolution
                motorSignalL = (short)(motorSignalR); //desiredWheelSpeedL

            }
            else if (centralLaserRange < 1000 - threshhold)
            {
                motorSignalR = (short)(-diff / gain); //desiredWheelSpeedL pulse/revolution
                motorSignalL = (short)(motorSignalR); //desiredWheelSpeedL

            }
            if (motorSignalL > 50)
            {
                motorSignalL = 50;
                motorSignalR = 50;
            }
            else if (motorSignalL < -50)
            {
                motorSignalL = -50;
                motorSignalR = -50;
            }

            // ****************** Additional Student Code: End   ************                
        }



        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            desiredT = ThetaWrapAround(desiredT);

            double deltax = desiredX - x_est;
            double deltay = desiredY - y_est;

            //            Console.WriteLine("Calc: {0} deltax: {1} deltay: {2}\n", Math.Pow(deltax, 2), deltax, deltay);

            double pho = Math.Sqrt(Math.Pow(deltax, 2) + Math.Pow(deltay, 2));
            double alpha = ThetaWrapAround(-t_est + Math.Atan2(deltay, deltax));
            //            Console.WriteLine("alpha: {0} calc2: {1}\n", alpha, Math.Atan2(deltay, deltax));
            double beta;

            bool isForward = true;
            double desiredV;
            double desiredW;

            double wheelCirc = wheelRadius * 2 * Math.PI;

            // Check if the robot should drive forwards
            if (Math.Abs(alpha) > Math.PI / 2)
            {
                //Backwards movement calculation
                alpha = ThetaWrapAround(-t_est + Math.Atan2(-deltay, -deltax));
                isForward = false;
            }

            double angleDiff = ThetaWrapAround(desiredT - t_est);

            //Console.WriteLine("angleDiff: {0}, desiredT: {1}, t_est: {2},", angleDiff, desiredT, t_est);

            beta = ThetaWrapAround(angleDiff - alpha);

            // Ensure that all angles are between -Pi and Pi
            //           alpha = ThetaWrapAround(alpha);
            //           beta = ThetaWrapAround(beta);

            // Checks if the robot is tracking trajectory
            if (isTrajectoryTracking)
            {
                //Checks to see if the robot is within the boundaries
                checkDist(pho);
            }

            //Console.WriteLine("pho: {0} alpha: {1} beta: {2}\n", pho, alpha, beta);

            // Check if the robot has reached the threshold displacement and the orientation
            if (Math.Abs(pho) < phoTrackingAccuracy && Math.Abs(angleDiff) < betaTrackingAccuracy)
            {
                desiredRotRateL = 0;
                desiredRotRateR = 0;

                Console.WriteLine("Stopped\n");
            }
            else if (Math.Abs(pho) < phoTrackingAccuracy)
            {
                double u = k_rot * angleDiff;

                //double u = k_rot * (desiredT - t_est) / (Math.Abs(desiredT - t_est));
                psi1 = u;
                psi2 = -u;

                capWheelRot();

                desiredRotRateL = (short)(psi2 / (2 * Math.PI) * ((double)pulsesPerRotation));
                desiredRotRateR = (short)(psi1 / (2 * Math.PI) * ((double)pulsesPerRotation));
                Console.WriteLine("Rotating\n");
            }

            else
            {
                //Error--------------------------------------------------------------------
                // Calculates the velocity and angular velocity based on the gain values
                // and if the robot should be moving forwards or backwards
                if (isForward)
                {
                    desiredV = Kpho * pho;
                    desiredW = Kalpha * alpha + Kbeta * beta;
                    //                    Console.WriteLine("Forward\n");
                }
                else
                {
                    desiredV = -Kpho * pho;
                    desiredW = Kalpha * alpha + Kbeta * beta;
                    //                    Console.WriteLine("Backward\n");
                }

                //Global Variables
                psi1 = (desiredV + robotRadius * desiredW) / wheelRadius;
                psi2 = (desiredV - robotRadius * desiredW) / wheelRadius;

                capWheelRot();

                desiredRotRateL = (short)(psi2 / (2 * Math.PI) * ((double)pulsesPerRotation));
                desiredRotRateR = (short)(psi1 / (2 * Math.PI) * ((double)pulsesPerRotation));

                //                desiredRotRateL = checkWheelRot(desiredRotRateL);
                //                desiredRotRateR = checkWheelRot(desiredRotRateR);



                //                Console.WriteLine("desiredV: {0} desiredW: {1}\n", desiredV, desiredW);
                //                Console.WriteLine("psi1: {0} psi2: {1}\n", psi1, psi2);
                //                Console.WriteLine("desiredRotRateL: {0} desiredRotRateR: {1}\n", desiredRotRateL, desiredRotRateR);

                //                desiredRotRateL = 0;
                //                desiredRotRateR = 0;
            }
        }

        //checks if the robot state is close enough to the target to move to the next point
        void checkDist(double phoTemp)
        {
            if (phoTemp < phoTrajectoryAccuracy)
            {
                Console.WriteLine("New Target");
                inRange = true;
            }
            else
            {
                inRange = false;
            }
        }

        // Calls global variable psi1 and psi2
        void capWheelRot()
        {
            // The maximum wheel rotation (rad/s) to get 0.25 m/s on the wheel
            double rotRateLimit = (maxVelocity / wheelRadius);
            if (Math.Abs(psi1) > rotRateLimit || Math.Abs(psi2) > rotRateLimit)
            {
                double maxScaling = Math.Max(Math.Abs(psi1), Math.Abs(psi2));
                psi1 = psi1 / maxScaling * rotRateLimit;
                psi2 = psi2 / maxScaling * rotRateLimit;
            }
        }


        // Ensure that all angles are between -Pi and Pi
        // If they are over, the angle values will rollover
        private double ThetaWrapAround(double angle)
        {
            while (Math.Abs(angle) > Math.PI)
            {
                if (angle > Math.PI)
                {
                    angle = angle - 2 * Math.PI;
                }
                else if (angle < -Math.PI)
                {
                    angle = angle + 2 * Math.PI;
                }
            }
            return angle;
        }


        // This function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            if (trajectoryState != numTrajectoryStates)
            {
                //Console.WriteLine("trajectoryState {0}",trajectoryState);
                //Console.WriteLine("desiredX: {0} desiredY: {1} desiredT: {2}\n", desiredX, desiredY, desiredT);
                if (inRange)
                {
                    Console.WriteLine("trajectoryState: {0}", trajectoryState);

                    desiredX = trajectoryArray[trajectoryState, 0];
                    desiredY = trajectoryArray[trajectoryState, 1];
                    desiredT = trajectoryArray[trajectoryState, 2];
                    trajectoryState = trajectoryState + 1;
                }
            }
        }

        // Initializes Trajectory Tracking
        void initTrajectory()
        {
            switch (trajectorySelect)
            {
                case 1:
                    trajectoryArray = trajLine;
                    break;
                case 2:
                    trajectoryArray = trajCircle;
                    break;
            }
            desiredX = trajectoryArray[trajectoryState, 0];
            desiredY = trajectoryArray[trajectoryState, 1];
            desiredT = trajectoryArray[trajectoryState, 2];
        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        // Calculates the difference between the current encoder setting with the previous one
        // accounting for possible rollover when the encoderMax has been exceeded
        public double encoderDifference(double currentEncoderPulse, double lastEncoderPulse)
        {


            double encoderThreshold = encoderMax * 3 / 4;
            double diffEncoderPulse = currentEncoderPulse - lastEncoderPulse;

            // Checks if the encoder has jumped more than is possible as determined by the
            // encoder threshhold which indicates that the encoder values have rolled over.
            if (Math.Abs(diffEncoderPulse) > encoderThreshold)
            {
                // Checks if the difference is positive or negative
                if (lastEncoderPulse > currentEncoderPulse)
                {
                    diffEncoderPulse = encoderMax + diffEncoderPulse;
                }
                else
                {
                    diffEncoderPulse = -encoderMax + diffEncoderPulse;
                }
            }

            return diffEncoderPulse;
        }

        public double calcWheelDist(double diffEncoder)
        {
            double wheelCirc = wheelRadius * 2 * Math.PI;

            return (diffEncoder / (double)pulsesPerRotation * wheelCirc);

        }

        public void MotionPrediction()
        {

            diffEncoderPulseR = encoderDifference(currentEncoderPulseR, lastEncoderPulseR);
            diffEncoderPulseL = encoderDifference(currentEncoderPulseL, lastEncoderPulseL);

            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;

            wheelDistanceR = calcWheelDist(-diffEncoderPulseR);
            wheelDistanceL = calcWheelDist(diffEncoderPulseL);

            distanceTravelled = (wheelDistanceR + wheelDistanceL) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);
            //Console.WriteLine("Entered Motion Prediction");
        }


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            double deltax = distanceTravelled * Math.Cos(t + angleTravelled / 2);
            double deltay = distanceTravelled * Math.Sin(t + angleTravelled / 2);

            x = x + deltax;
            y = y + deltay;
            t = t + angleTravelled;

            if (t > Math.PI)
            {
                t = t - 2 * Math.PI;
            }
            else if (t < -Math.PI)
            {
                t = t + 2 * Math.PI;
            }
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF
            double maxParticleWeight = 0;
            if (distanceTravelled != 0 || angleTravelled != 0)
            {
               // newLaserData = false;
                for (int i = 0; i < numParticles; ++i)
                {
                    //(1 - random.NextDouble() * K_wheelRandomness + K_wheelRandomness/2)
                    double wheelDistanceRTemp = wheelDistanceR + wheelDistanceR * RandomGaussian() * K_wheelRandomness;
                    double wheelDistanceLTemp = wheelDistanceL + wheelDistanceL * RandomGaussian() * K_wheelRandomness;

                    double distanceTravelledTemp = (wheelDistanceRTemp + wheelDistanceLTemp) / 2;
                    double angleTravelledTemp = (wheelDistanceRTemp - wheelDistanceLTemp) / (2 * robotRadius);

                    propagatedParticles[i].x = particles[i].x + distanceTravelledTemp * Math.Cos(particles[i].t + angleTravelledTemp / 2);
                    propagatedParticles[i].y = particles[i].y + distanceTravelledTemp * Math.Sin(particles[i].t + angleTravelledTemp / 2);
                    propagatedParticles[i].t = ThetaWrapAround(particles[i].t + angleTravelledTemp);

                    CalculateWeight(i);
                    if(maxParticleWeight < propagatedParticles[i].w)
                    {
                        maxParticleWeight = propagatedParticles[i].w;
                    }
   //                 Console.Write("Particle Number: {0}, Location: ({1},{2},{3}), ", i, propagatedParticles[i].x, propagatedParticles[i].y, propagatedParticles[i].t);
   //                 Console.WriteLine("Weight: {0}, MaxWeight {1} ", propagatedParticles[i].w, maxParticleWeight);
                    
                }
            
                resampleParticles(maxParticleWeight);

                calcStateFromParticles();
            }

            // ****************** Additional Student Code: End   ************

        }

        void resampleParticles(double maxWeight)
        {
            int[] tempParticles = new int[4 * numParticles];
            int currPart = 0;
            for (int i = 0; i < numParticles; ++i)
            {
                double normalizedWeight = propagatedParticles[i].w / maxWeight;
                if (normalizedWeight < .25)
                {
                    tempParticles[currPart] = i;
                    currPart++;
                }
                else if (normalizedWeight < .5)
                {
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                }
                else if (normalizedWeight < .75)
                {
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                }
                else if (normalizedWeight <= 1)
                {
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                    tempParticles[currPart] = i;
                    currPart++;
                }
            }
            for (int i = 0; i < numParticles; ++i)
            {
                int selectPart = (int)(random.NextDouble() * (double)currPart);
                particles[i].x = propagatedParticles[tempParticles[selectPart]].x;
                particles[i].y = propagatedParticles[tempParticles[selectPart]].y;
                particles[i].t = propagatedParticles[tempParticles[selectPart]].t;
            }
        }
        
        // Sets the state estimate as the average of all the particles
        void calcStateFromParticles()
        {
            double sumX = 0;
            double sumY = 0;
            double sumT = 0;
            for (int i = 0; i < numParticles; ++i)
            {
                sumX = sumX + particles[i].x;
                sumY = sumY + particles[i].y;
                sumT = sumT + particles[i].t;
            }
            x_est = sumX / numParticles;
            y_est = sumY / numParticles;
            t_est = sumT / numParticles;
        }


        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.
        void CalculateWeight(int p)
        {
            propagatedParticles[p].w = 0;
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                double partDist = 1000 * map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t - 1.57 + laserAngles[i]);
//                double temp = LaserData[i];
                double individualWeight = gaussianDist(partDist, LaserData[i], stdDevGuess);
                propagatedParticles[p].w = propagatedParticles[p].w + individualWeight;
                
              //  Console.WriteLine("Particle Distribution: {0},LaserData: {1},Gaussian: {2}",partDist,LaserData[i], gaussianDist(partDist, LaserData[i], stdDevGuess));
            }
        }

        double gaussianDist(double value,double mean, double stdDev)
        {
            return Math.Exp(-1*Math.Pow((value - mean),2)/(2*Math.Pow(stdDev,2)));
        }

        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; ++i){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
                {
                    SetRandomPos(i);
                }
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
                {
                    SetStartPos(i);
                }
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){
            particles[p].x = random.NextDouble() * (map.maxX - map.minX) + map.minX;
            particles[p].y = random.NextDouble() * (map.maxY - map.minY) + map.minY;
            particles[p].t = 2 * Math.PI * random.NextDouble() - Math.PI;
            particles[p].w = 1.0 / numParticles;
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
            // Set to non-zero for kidnapped Robot scenario
            double offset = 0;
	        particles[p].x = initialX + offset;
	        particles[p].y = initialY + offset;
	        particles[p].t = initialT + offset;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
