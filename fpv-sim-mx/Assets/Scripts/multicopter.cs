using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;
using MathFunctions;



public class multicopter : MonoBehaviour
{
    motor[] motors;
    bool armed; //Armed variable
    private string debugInfo; //:) Display Debug variable
    [Range(0.0f, 1.0f)]
    public float throttlePos = 0f;

    Transmitter tx;
    Sensors sensors;
    CFController controller;

    // GUIText is used for tests
    public Text display;


    // Use this for initialization
    void Start()
    {
        //Multicopter motor configuration
        //Quadcopter
        motors = new motor[4];
        motors[0] = new motor(new Vector3(1, 0, 1), 0f, true); //Front left CW
        motors[1] = new motor(new Vector3(1, 0, -1), 0f, false); //Front righ CCW
        motors[2] = new motor(new Vector3(-1, 0, -1), 0f, true); //Rear right CW
        motors[3] = new motor(new Vector3(-1, 0, 1), 0, false); //Rear left CCW

        tx = new Transmitter();
        sensors = new Sensors(GetComponent<Rigidbody>());
        controller = new CFController(0);

        armed = true;
    }

    // Update is called once per frame but not used here because its timing depends on the CPU speed
    //void Update(){
    //}

    // This is a time dependent update
    void FixedUpdate()
    {

        float torqueCW = 0;
        float torqueCCW = 0;

        //Calls the controller
        controller.ControlUpdate(motors, tx, sensors);
        armed = controller.armed;

        //Calls the sound updater
        //if (armed)
        //    sound.updateSound(tx.GetThrottle());


        //Apply the force to the motors
        for (int i = 0; i < 4; i++)
        {
            GetComponent<Rigidbody>().AddForceAtPosition(transform.up * motors[i].force, transform.position + Quaternion.Euler(sensors.GetRotation()) * motors[i].position, ForceMode.Force);
            if (motors[i].clockwise)
                torqueCW += motors[i].force;
            else
                torqueCCW += motors[i].force;
        }

        //Changes variable for animation (not used in FPV version, only in Aerial Photography)
        /*GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M1CCW").gameObject.GetComponent<propspinner>().motorPower = motors[0].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M2CW").gameObject.GetComponent<propspinner>().motorPower = motors[1].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M3CCW").gameObject.GetComponent<propspinner>().motorPower = motors[2].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M4CW").gameObject.GetComponent<propspinner>().motorPower = motors[3].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M5CCW").gameObject.GetComponent<propspinner>().motorPower = motors[4].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M6CW").gameObject.GetComponent<propspinner>().motorPower = motors[5].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M7CCW").gameObject.GetComponent<propspinner>().motorPower = motors[6].force;
        GetComponent<Rigidbody>().transform.FindChild("Airframe").gameObject.transform.FindChild("M8CW").gameObject.GetComponent<propspinner>().motorPower = motors[7].force;
        */
        //Apply torque :) Temporary function
        if (armed && tx.GetThrottle() > 0.02F)
            GetComponent<Rigidbody>().AddRelativeTorque(0, 5 * tx.GetYaw()/*torqueCW-torqueCCW*/, 0, ForceMode.Force);
        else
            GetComponent<Rigidbody>().AddRelativeTorque(0, 0, 0, ForceMode.Force);

        //Display info
//        		display.text = "dadada";
        		display.text = "Roll: \t" + tx.GetRoll();
        		display.text += "\nPitch: \t" + tx.GetPitch();
        		display.text += "\nYaw: \t" + tx.GetYaw();
        		display.text += "\nThrottle: \t" + tx.GetThrottle();
        //:)display.text += "\nTilt: \t" + tx.GetTilt();
        //:)display.text += "\nTorsion total: " + (torqueCW - torqueCCW);
        		for (int i=0; i<4; i++)
        			display.text += "\nFuerza motor " +i+ ": " + motors[i].force;
        //		display.text += "\nAltitude: "+ sensors.GetAltitude();
        //		display.text += "\nAngular velocity: " + sensors.GetAngularVelocity();
        //		display.text += "\n" + debugInfo;
        //Debug.Log("Throttle " + tx.GetThrottle());
    }
}


public class CFController //Based on Cleanflight controller
{

    public const double RCconstPI = 0.159154943092f;

    //Modes
    [Flags] public enum Modes {
        ANGLE_MODE = 0,
        HORIZON_MODE = 1,
        MAG_MODE = 2,
        BARO_MODE = 3,
        GPS_HOME_MODE = 4,
        GPS_HOLD_MODE = 5,
        HEADFREE_MODE = 6,
        AUTOTUNE_MODE = 7,
        PASSTHRU_MODE = 8,
        SONAR_MODE = 9,
        FAILSAFE_MODE = 10
    }

    enum rc_alias
    {
	    ROLL = 0,
	    PITCH,
	    YAW,
	    THROTTLE,
	    AUX1,
	    AUX2,
	    AUX3,
        AUX4,
        AUX5,
        AUX6,
        AUX7,
        AUX8
    }

    enum pidIndex_e
    {
        PIDROLL,
        PIDPITCH,
        PIDYAW,
        PIDALT,
        PIDPOS,
        PIDPOSR,
        PIDNAVR,
        PIDLEVEL,
        PIDMAG,
        PIDVEL,
        PID_ITEM_COUNT
    }

    static Int32[] errorGyroI = new Int32[3] { 0, 0, 0 };
    static float[] errorGyroIf = new float[3]{ 0.0f, 0.0f, 0.0f };
    static Int32[] errorAngleI = new Int32[2]{ 0, 0 };
    static float[] errorAngleIf = new float[2]{ 0.0f, 0.0f };

    void pidResetErrorAngle()
    {
        errorAngleI[(int)rc_alias.ROLL] = 0;
        errorAngleI[(int)rc_alias.PITCH] = 0;

        errorAngleIf[(int)rc_alias.ROLL] = 0.0f;
        errorAngleIf[(int)rc_alias.PITCH] = 0.0f;
    }

    void pidResetErrorGyro()
    {
        errorGyroI[(int)rc_alias.ROLL] = 0;
        errorGyroI[(int)rc_alias.PITCH] = 0;
        errorGyroI[(int)rc_alias.YAW] = 0;

        errorGyroIf[(int)rc_alias.ROLL] = 0.0f;
        errorGyroIf[(int)rc_alias.PITCH] = 0.0f;
        errorGyroIf[(int)rc_alias.YAW] = 0.0f;
    }

    class pidProfileClass{
        byte pidController;                  // 0 = multiwii original, 1 = rewrite from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671, 1, 2 = Luggi09s new baseflight pid

        byte[] P8 = new byte[(int)pidIndex_e.PID_ITEM_COUNT];
        byte[] I8 = new byte[(int)pidIndex_e.PID_ITEM_COUNT];
        byte[] D8 = new byte[(int)pidIndex_e.PID_ITEM_COUNT];

        float[] P_f = new float[3];                           // float p i and d factors for lux float pid controller
        float[] I_f = new float[3];
        float[] D_f = new float[3];
        float A_level;
        float H_level;
        byte H_sensitivity;

        UInt16 yaw_p_limit;                   // set P term limit (fixed value was 300)
        byte dterm_cut_hz;                   // (default 17Hz, Range 1-50Hz) Used for PT1 element in PID1, PID2 and PID5
        byte pterm_cut_hz;                   // Used for fitlering Pterm noise on noisy frames
        byte gyro_cut_hz;                    // Used for soft gyro filtering

        byte pid5_oldyw;                     // [0/1] 0 = multiwii 2.3 yaw, 1 = older yaw

# if GTUNE
        byte gtune_lolimP[3];               // [0..200] Lower limit of P during G tune
        byte gtune_hilimP[3];               // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
        byte gtune_pwr;                     // [0..10] Strength of adjustment
        UInt16 gtune_settle_time;             // [200..1000] Settle time in ms
        byte gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
#endif
    }

    // ________ Being translated from Cleanflight
    static void pidHarakiri()
    {
        MFuncts mathfunct = new MFuncts();
        //(void)(rxConfig);

        float delta, RCfactor, rcCommandAxis, MainDptCut, gyroADCQuant;
        float PTerm, ITerm, DTerm, PTermACC = 0.0f, ITermACC = 0.0f, ITermGYRO, error, prop = 0.0f;
        float[] lastGyro = new float[2]{ 0.0f, 0.0f }, lastDTerm = new float[2]{ 0.0f, 0.0f };
        //uint8_t axis;
        byte axis;
        float ACCDeltaTimeINS, FLOATcycleTime, Mwii3msTimescale;

        MainDptCut = RCconstPI / mathfunct.constrain(pidProfile->dterm_cut_hz, 1, 50);       // maincuthz (default 0 (disabled), Range 1-50Hz)
        FLOATcycleTime = (float)constrain(cycleTime, 1, 100000);                  // 1us - 100ms
        ACCDeltaTimeINS = FLOATcycleTime * 0.000001f;                              // ACCDeltaTimeINS is in seconds now
        RCfactor = ACCDeltaTimeINS / (MainDptCut + ACCDeltaTimeINS);               // used for pt1 element

        if (FLIGHT_MODE(HORIZON_MODE))
        {
            prop = (float)MIN(MAX(ABS(rcCommand[PITCH]), ABS(rcCommand[ROLL])), 450) / 450.0f;
        }

        for (axis = 0; axis < 2; axis++)
        {
            int32_t tmp = (int32_t)((float)gyroADC[axis] * 0.3125f);              // Multiwii masks out the last 2 bits, this has the same idea
            gyroADCQuant = (float)tmp * 3.2f;                                     // but delivers more accuracy and also reduces jittery flight
            rcCommandAxis = (float)rcCommand[axis];                                // Calculate common values for pid controllers
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))
            {
# if GPS
                error = constrain(2.0f * rcCommandAxis + GPS_angle[axis], -((int)max_angle_inclination), +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis];
#else
                error = constrain(2.0f * rcCommandAxis, -((int)max_angle_inclination), +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis];
#endif

# if AUTOTUNE
                if (shouldAutotune())
                {
                    error = DEGREES_TO_DECIDEGREES(autotune(rcAliasToAngleIndexMap[axis], &inclination, DECIDEGREES_TO_DEGREES(error)));
                }
#endif
                PTermACC = error * (float)pidProfile->P8[PIDLEVEL] * 0.008f;
                float limitf = (float)pidProfile->D8[PIDLEVEL] * 5.0f;
                PTermACC = constrain(PTermACC, -limitf, +limitf);
                errorAngleIf[axis] = constrainf(errorAngleIf[axis] + error * ACCDeltaTimeINS, -30.0f, +30.0f);
                ITermACC = errorAngleIf[axis] * (float)pidProfile->I8[PIDLEVEL] * 0.08f;
            }

            if (!FLIGHT_MODE(ANGLE_MODE))
            {
                if (ABS((int16_t)gyroADC[axis]) > 2560)
                {
                    errorGyroIf[axis] = 0.0f;
                }
                else
                {
                    error = (rcCommandAxis * 320.0f / (float)pidProfile->P8[axis]) - gyroADCQuant;
                    errorGyroIf[axis] = constrainf(errorGyroIf[axis] + error * ACCDeltaTimeINS, -192.0f, +192.0f);
                }

                ITermGYRO = errorGyroIf[axis] * (float)pidProfile->I8[axis] * 0.01f;

                if (FLIGHT_MODE(HORIZON_MODE))
                {
                    PTerm = PTermACC + prop * (rcCommandAxis - PTermACC);
                    ITerm = ITermACC + prop * (ITermGYRO - ITermACC);
                }
                else
                {
                    PTerm = rcCommandAxis;
                    ITerm = ITermGYRO;
                }
            }
            else
            {
                PTerm = PTermACC;
                ITerm = ITermACC;
            }

            PTerm -= gyroADCQuant * dynP8[axis] * 0.003f;

            // Pterm low pass
            if (pidProfile->pterm_cut_hz)
            {
                PTerm = filterApplyPt1(PTerm, &PTermState[axis], pidProfile->pterm_cut_hz);
            }

            delta = (gyroADCQuant - lastGyro[axis]) / ACCDeltaTimeINS;

            lastGyro[axis] = gyroADCQuant;
            lastDTerm[axis] += RCfactor * (delta - lastDTerm[axis]);
            DTerm = lastDTerm[axis] * dynD8[axis] * 0.00007f;

            axisPID[axis] = lrintf(PTerm + ITerm - DTerm);                         // Round up result.

# if BLACKBOX
            axisPID_P[axis] = PTerm;
            axisPID_I[axis] = ITerm;
            axisPID_D[axis] = -DTerm;
#endif
        }

        Mwii3msTimescale = (int32_t)FLOATcycleTime & (int32_t)~3;                  // Filter last 2 bit jitter
        Mwii3msTimescale /= 3000.0f;

        if (OLD_YAW)
        { // [0/1] 0 = multiwii 2.3 yaw, 1 = older yaw. hardcoded for now
            PTerm = ((int32_t)pidProfile->P8[FD_YAW] * (100 - (int32_t)controlRateConfig->rates[FD_YAW] * (int32_t)ABS(rcCommand[FD_YAW]) / 500)) / 100;
            int32_t tmp = lrintf(gyroADC[FD_YAW] * 0.25f);
            PTerm = rcCommand[FD_YAW] - tmp * PTerm / 80;
            if ((ABS(tmp) > 640) || (ABS(rcCommand[FD_YAW]) > 100))
            {
                errorGyroI[FD_YAW] = 0;
            }
            else
            {
                error = ((int32_t)rcCommand[FD_YAW] * 80 / (int32_t)pidProfile->P8[FD_YAW]) - tmp;
                errorGyroI[FD_YAW] = constrain(errorGyroI[FD_YAW] + (int32_t)(error * Mwii3msTimescale), -16000, +16000); // WindUp
                ITerm = (errorGyroI[FD_YAW] / 125 * pidProfile->I8[FD_YAW]) >> 6;
            }
        }
        else
        {
            int32_t tmp = ((int32_t)rcCommand[FD_YAW] * (((int32_t)controlRateConfig->rates[FD_YAW] << 1) + 40)) >> 5;
            error = tmp - lrintf(gyroADC[FD_YAW] * 0.25f);                       // Less Gyrojitter works actually better

            if (ABS(tmp) > 50)
            {
                errorGyroI[FD_YAW] = 0;
            }
            else
            {
                errorGyroI[FD_YAW] = constrain(errorGyroI[FD_YAW] + (int32_t)(error * (float)pidProfile->I8[FD_YAW] * Mwii3msTimescale), -268435454, +268435454);
            }

            ITerm = constrain(errorGyroI[FD_YAW] >> 13, -GYRO_I_MAX, +GYRO_I_MAX);
            PTerm = ((int32_t)error * (int32_t)pidProfile->P8[FD_YAW]) >> 6;

            if (motorCount >= 4 && pidProfile->yaw_p_limit < YAW_P_LIMIT_MAX)
            {     // Constrain FD_YAW by D value if not servo driven in that case servolimits apply
                PTerm = constrain(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
            }
        }

        // Pterm low pass
        if (pidProfile->pterm_cut_hz)
        {
            PTerm = filterApplyPt1(PTerm, &PTermState[axis], pidProfile->pterm_cut_hz);
        }

        axisPID[FD_YAW] = PTerm + ITerm;
        axisPID[FD_YAW] = lrintf(axisPID[FD_YAW]);                                 // Round up result.

# if BLACKBOX
        axisPID_P[FD_YAW] = PTerm;
        axisPID_I[FD_YAW] = ITerm;
        axisPID_D[FD_YAW] = 0;
#endif
    }


    //________________________ Ends code being translated from Cleanflight
    public int currMode = 0;
    private CtrlChannel pitch;
    private CtrlChannel roll;
    private CtrlChannel yaw;
    private CtrlChannel throttle;
    public bool armed;
    private int contCyclesBeforeArmUnarm;

    public CFController(Modes modes)
    {
        if ((modes & Modes.ANGLE_MODE)!=0)
        {
        }

        roll = new CtrlChannel(4500, 5000, 5, 0.0f, 0.5f, 0.0f, 0f);
        pitch = new CtrlChannel(4500, 5000, 5, 0.0f, 0.5f, 0.0f, 0f);
        yaw = new CtrlChannel(4500, 5000, 5, 2.0f, 1.0f, 0.0f, 0.0f);

        armed = true;
        contCyclesBeforeArmUnarm = 0;
    }


    public void ControlUpdate(motor[] motors, Transmitter tx, Sensors sensors)
    {
        if (armed)
        {
            if (Modes.BARO_MODE)
            {//35 degrees max inclination
                roll.Control(sensors.GetRotation().x, sensors.GetAngularVelocity().x, -tx.GetRoll() * 35);
                pitch.Control(sensors.GetRotation().z, sensors.GetAngularVelocity().z, -tx.GetPitch() * 35);
                yaw.Control(sensors.GetRotation().y, sensors.GetAngularVelocity().y, tx.GetYaw() * 50);
            }
            //Calculates and applies the force to the motors
            for (int i = 0; i < 4; i++)
            {
                //General force (throttle)
                motors[i].force = tx.GetThrottle() * 2.0F; //:) This should take into account the air density, air temp, altitude, batteries charge, etc.
                                                           //Pitch force
                motors[i].force -= motors[i].position.x * (pitch.p + pitch.i + pitch.d);
                //Roll force
                motors[i].force += motors[i].position.z * (roll.p + roll.i + roll.d);// + (-0.1F + Random.value*0.2F);
            }
            if (tx.GetYaw() < -0.9 && tx.GetThrottle() < 0.1)
            {
                contCyclesBeforeArmUnarm++;
            }
            else
            {
                contCyclesBeforeArmUnarm = 0;
            }
            if (contCyclesBeforeArmUnarm >= 100)
            {
                armed = false;
                Debug.Log("UNARMED");
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                motors[i].force = 0f;
            }
            if (tx.GetYaw() > 0.9 && tx.GetThrottle() < 0.1)
            {
                contCyclesBeforeArmUnarm++;
            }
            else
            {
                contCyclesBeforeArmUnarm = 0;
            }
            if (contCyclesBeforeArmUnarm >= 100)
            {
                armed = true;
                Debug.Log("ARMED");
            }
        }
    }

    public void ChangeMode(int newMode)
    {
        currMode = newMode;
    }

    private class Angle
    {
        float sinVal;
        float cosVal;

        public Angle(float angleDeg)
        { //initializes angle given in degrees
            sinVal = Mathf.Sin(Mathf.Deg2Rad * angleDeg);
            cosVal = Mathf.Cos(Mathf.Deg2Rad * angleDeg);
        }

        public float ValAngle()
        { //return angle in degrees normalizing between -180 and 180
            return Mathf.Rad2Deg * Mathf.Atan2(sinVal, cosVal);
        }

        public void UpdateTo(float angleDeg)
        {
            sinVal = Mathf.Sin(Mathf.Deg2Rad * angleDeg);
            cosVal = Mathf.Cos(Mathf.Deg2Rad * angleDeg);
        }
    }

    private class Gains
    {
        public float kp;
        public float ki;
        public float kd;

        public Gains(float kP, float kI, float kD)
        {
            kp = kP;
            ki = kI;
            kd = kD;
        }
    }

    private class CtrlChannel
    {
        //rate Vars
        public Gains rGains;
        public float rIMax;
        public float rTarget;
        public float rCurrent;
        public float rError;
        public float rErrorI = 0;
        public float rErrorD = 0;
        private float rErrorPrev = 0;
        public float rLimit;
        //Stable Vars
        public Gains sGains;
        public Angle sTarget;
        public Angle sCurrent;
        public float sLimit;
        public float sError;
        //Control components
        public float p;
        public float i;
        public float d;

        public CtrlChannel(float sLim, float rLim, float rImx, float sKp, float rKp, float rKi, float rKd)
        { // :) We should initialize all important variables (Limits, gains, imax)
            sLimit = sLim;
            rLimit = rLim;
            sGains = new Gains(sKp, 0, 0);
            rGains = new Gains(rKp, rKi, rKd);
            sTarget = new Angle(0);
            sCurrent = new Angle(0);
            rIMax = rImx;
        }

        public void Control(float currentAtt, float currentRate, float target)
        {//Calculates p,i,d components
            sCurrent.UpdateTo(currentAtt); //Measured attitude
            sTarget.UpdateTo(target); //Target from stick position
            sError = sCurrent.ValAngle() - sTarget.ValAngle();
            if (Mathf.Abs(sError) > sLimit) //Limits the error
                sError = Mathf.Sign(sError) * sLimit;

            RateControl(currentRate, sGains.kp * sError);
            //Debug.Log("currentRate: " + currentRate);
            //Debug.Log("sError: "+sError);
        }

        private void RateControl(float current, float target)
        {
            rCurrent = current;
            rTarget = target;
            rError = rTarget - rCurrent;
            rErrorI = rErrorI + rError;
            if (Mathf.Abs(rErrorI) > rIMax)
                rErrorI = Mathf.Sign(rErrorI) * rIMax;
            rErrorD = rError - rErrorPrev;
            rErrorPrev = rError;

            p = rGains.kp * rError;
            i = rGains.ki * rErrorI;
            d = rGains.kd * rErrorD;
            //Debug.Log("p:"+p);
            //Debug.Log("i:"+i);
            //Debug.Log("d:"+d);
        }

    }
}

public class Sensors
{
    Vector3 rotation; //accelerometers + gyroscopes
    Vector3 angularVelocity; //gyroscopes
    Vector3 acceleration; //accelerometers
    Vector3 orientation; // magnetometers
    float altitude; //barometer
    float initialAltitude;

    Rigidbody rb;

    public Sensors(Rigidbody r)
    {
        rb = r;
        initialAltitude = rb.position.y;
    }

    public float GetAltitude()
    {
        altitude = rb.position.y - initialAltitude;
        return altitude;
    }

    public Vector3 GetRotation()
    {
        rotation = rb.rotation.eulerAngles;
        return rotation;
    }

    public Vector3 GetAngularVelocity()
    {
        angularVelocity = rb.angularVelocity;
        return angularVelocity;
    }
}

public class Transmitter
{
    /*
	 *  Filters and conditions transmitter's signal to generate inputs for the controller algorithm.
	 */
    //Values after filtering and range adjustment
    private float roll; // in [-1,1] = [left, right]
    private float pitch; // in in [-1,1] = [nose up, nose down]
    private float yaw; // in in [-1,1] = [left, right]
    private float throttle; // in [0,1] = [low, high]
    private float tilt; // in[-1,1] = [tiltdown, tiltup]{speed} OR [-90, 0]{position}

    //Max/mins of input signal as sent by the joystick driver. This is used for calibration
    private float minroll = -0.7960784F;
    private float minpitch = -0.8352F;
    private float minyaw = -1;
    private float minThrottle = -1;//-0.9764706F;
    private float maxroll = 0.866666F;
    private float maxpitch = 0.73333F;
    private float maxyaw = 1;
    private float maxThrottle = 1;
    private float minTilt = -1;
    private float maxTilt = 1;


    private float alpha = 0.2F; //Filter coefficient

    public Transmitter()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
        throttle = 0;
        tilt = 0;
    }

    public float GetThrottle()
    { //This should return a value between 0 and 1
      //:)
      //if(throttle<1)
      //	throttle += 0.0001f; //:) throttle * (1.0F - alpha) + ((Input.GetAxis("MRThrottle")-minThrottle)*alpha)/(maxThrottle-minThrottle);
      //:)
        throttle = throttle * (1.0F - alpha) + ((Input.GetAxis("MRThrottle") - minThrottle) * alpha) / (maxThrottle - minThrottle);
        return throttle;
    }

    public float GetRoll()
    { //This should return a value between -1 and 1
      //		roll = 0; //:)roll*(1.0F - alpha) + Input.GetAxis("MRRoll")*alpha;
        roll = roll * (1.0F - alpha) + Input.GetAxis("MRRoll") * alpha;
        return roll;
    }

    public float GetPitch()
    { //This should return a value between -1 and 1
      //		pitch = 0; //:) pitch*(1.0F - alpha) + Input.GetAxis("MRPitch")*alpha;
        pitch = pitch * (1.0F - alpha) + Input.GetAxis("MRPitch") * alpha;
        return pitch;
    }

    public float GetYaw()
    { //This should return a value between -1 and 1
        yaw = yaw * (1.0F - alpha) + Input.GetAxis("MRYaw") * alpha;
        return yaw;
    }

    //:)
    //	public float GetTilt(){
    //		tilt = tilt * (1.0F - alpha) + Input.GetAxis("CameraTilt")*alpha;
    //		return tilt;
    //	}
    //

}

public class motor
{
    public Vector3 position; //relative position to the center of mass of the aircraft
    public float force; // THE force
    public bool clockwise; // Specifies wether the motor turns clockwise or couterclockwise

    public motor(Vector3 Vposition, float frce, bool cwise)
    {
        position = Vposition;
        force = frce;
        clockwise = cwise;
    }
}

