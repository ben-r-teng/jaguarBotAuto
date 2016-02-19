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
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 4;//4
        private double Kbeta = -.5;//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.05;
        const double phoTrackingAccuracy = 0.05;
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

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
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
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
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
                // functions to call here. For lab 3, we just call the function
                FlyToSetPoint();
                
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

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Determine the desired PWM signals for desired wheel speeds
                    CalcMotorSignals();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    ActuateMotorsWithPWMControl();
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
        // the PWM signal for corresponding desired wheel speeds.
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // ****************** Additional Student Code: Start ************

            // Students must set motorSignalL and motorSignalR. Make sure
            // they are set between 0 and maxPosOutput. A PID control is
            // suggested.

            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            // ****************** Additional Student Code: End   ************

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
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
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
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString();

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



            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            double deltax = desiredX - x_est;
            double deltay = desiredY - y_est;

            Console.WriteLine("Calc: {0} deltax: {1} deltay: {2}\n", Math.Pow(deltax, 2), deltax, deltay);

            double pho = Math.Sqrt(Math.Pow(deltax, 2) + Math.Pow(deltay, 2));
            double alpha = -desiredT + Math.Atan2(deltay, deltax);
            Console.WriteLine("alpha: {0} calc2: {1}\n", alpha, Math.Atan2(deltay, deltax));
            double beta;
            
            bool isForward = true;
            double desiredV;
            double desiredW;

            double wheelCirc = wheelRadius * 2 * Math.PI;

            // Check if the robot should drive forwards
            if (Math.Abs(alpha) > Math.PI / 2)
            {
                //Backwards movement calculation
                alpha = -desiredT + Math.Atan2(-deltay, -deltax);
                isForward = false;
            }

            beta = -t_est - alpha;

            // Ensure that all angles are between -Pi and Pi
            alpha = ThetaWrapAround(alpha);
            beta = ThetaWrapAround(beta);

            Console.WriteLine("pho: {0} alpha: {1} beta: {2}\n", pho, alpha, beta);

            // Check if the robot has reached the threshold displacement and the orientation
            if (Math.Abs(pho) < phoTrackingAccuracy && Math.Abs(beta) < betaTrackingAccuracy)
            {
                desiredRotRateL = 0;
                desiredRotRateR = 0;
                Console.WriteLine("Stopped\n");
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
                    Console.WriteLine("Forward\n");
                }
                else
                {
                    desiredV = -Kpho * pho;
                    desiredW = Kalpha * alpha + Kbeta * beta;
                    Console.WriteLine("Backward\n");
                }


/*
                double w1 = (desiredW + desiredV / robotRadius) / 2;
                double w2 = (desiredW - desiredV / robotRadius) / 2;

                double psi1 = 2 * robotRadius / wheelRadius * w1;
                double psi2 = -2 * robotRadius / wheelRadius * w2;
*/
                double psi1 = (desiredV + robotRadius * desiredW) / wheelRadius;
                double psi2 = (desiredV - robotRadius * desiredW) / wheelRadius;
                

                desiredRotRateL = (short)(psi2 / wheelCirc * ((double)pulsesPerRotation));
                desiredRotRateR = (short)(psi1 / wheelCirc * ((double)pulsesPerRotation));

                desiredRotRateL = checkWheelRot(desiredRotRateL);
                desiredRotRateR = checkWheelRot(desiredRotRateR);

//                capWheelRot(desiredRotRateL, desiredRotRateR);

                Console.WriteLine("desiredV: {0} desiredW: {1}\n", desiredV, desiredW);
                Console.WriteLine("psi1: {0} psi2: {1}\n", psi1, psi2);
                Console.WriteLine("desiredRotRateL: {0} desiredRotRateR: {1}\n", desiredRotRateL, desiredRotRateR);

//                desiredRotRateL = 0;
//                desiredRotRateR = 0;
            }
        }

        // Sets the upper limit of the wheel rotation speed
        short checkWheelRot(short desiredRotRate)
        {
            short rotRateLimit = 150;
            if (desiredRotRate > rotRateLimit)
            {
                desiredRotRate = rotRateLimit;
            }
            else if (desiredRotRate < ((short)-rotRateLimit))
            {
                desiredRotRate = ((short)-rotRateLimit);
            }
            return desiredRotRate;
        }

        void capWheelRot(short rotRateL, short rotRateR)
        {
            short rotRateLimit = 150;
            if (Math.Abs(rotRateL) > rotRateLimit || Math.Abs(rotRateR) > rotRateLimit)
            {
                double maxScaling = Math.Max(Math.Abs(rotRateL), Math.Abs(rotRateR));
                desiredRotRateL = (short) ((double)(rotRateL) / maxScaling * (double)(rotRateLimit));
                desiredRotRateR = (short) ((double)(rotRateR) / maxScaling * (double)(rotRateLimit));
            }
        }


        // Ensure that all angles are between -Pi and Pi
        // If they are over, the angle values will rollover
        private double ThetaWrapAround(double angle)
        {
            if (angle > Math.PI)
            {
                angle = angle - 2 * Math.PI;
            }
            else if (angle < -Math.PI)
            {
                angle = angle + 2 * Math.PI;
            }
            return angle;
        }

        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

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

        public double encoderDifference(double currentEncoderPulse, double lastEncoderPulse)
        {

            double encoderThreshold = encoderMax * 3 / 4;
            double diffEncoderPulse = currentEncoderPulse - lastEncoderPulse;

            if (Math.Abs(diffEncoderPulse) > encoderThreshold)
            {
                if (lastEncoderPulse > currentEncoderPulse)
                {
                    diffEncoderPulse = encoderMax - lastEncoderPulse + currentEncoderPulse;
                }
                else
                {
                    diffEncoderPulse = -encoderMax + currentEncoderPulse - lastEncoderPulse;
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

            double diffEncoderPulseR = encoderDifference(currentEncoderPulseR, lastEncoderPulseR);
            double diffEncoderPulseL = encoderDifference(currentEncoderPulseL, lastEncoderPulseL);

            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;

            wheelDistanceR = calcWheelDist(-diffEncoderPulseR);
            wheelDistanceL = calcWheelDist(diffEncoderPulseL);

            distanceTravelled = (wheelDistanceR + wheelDistanceL) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);

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


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}
