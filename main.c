/*	SW-COMPUTER Factory Control Program
	WWW.EXTREME-FIRE.COM
	Target controller = ATTINY85-20PU microcontroller from ATMEL.
	Clock speed = 8 MHz internal.
	Development platform = AVR Studio 4  Version 4.13 Service Pack 2  Build 571.
	C compiler = WinAVR 20070525
	Programer = AVR Dragon
	Hardware Schematic = SW-COMPUTER all versions.
	Hardware Layout = SW-COMPUTER all versions.

This program is fully released to the Public Domain by Terry Fritz 2007 / 2008.
*/
/*
History
	001 - Initial release  Nov. 23, 2007  Terry Fritz
	002 - Next generation  Nov. 25, 2007  Terry Fritz
	002.40 - First real beta release.
	002.41 - Fixed Vibration timing problems, set peak current to 250 amps.	Dec. 26, 2007  9:24AM
	002.42 - Added Temperature sensing.  Dec. 26, 2007  1:10PM
	002.43 - Removed high battery voltage speed reduction.  High battery voltage to 17.0V.  Dec. 27, 2007 11:16PM
				Timing rechecked Dec 28, 2007 2:19PM
	002.44 - Added event logging, LiPO monitor support, PWM and BurstTimeFactor low limits.  Dec. 30, 2007 10:00AM
	002.45 - Added LiPO Monitoring. Redid Port control. Jan 1, 2008  8:51AM
	002.46 - Speeded up programing vibrations and Burst to Full auto time.  Check LiPO alert 3 times. Jan. 7, 2008 4:30PM
	002.50 - Released
	002.60 - Tweaked LiPO Function.  Lowered trip points from 3000mV to 1000mV.  Changed installed detection from
				2000mV to 750mV.  Feb. 15, 2008  11:10PM
			Added FiringCounter (was unused MotorRunningLoopCounter).
			Added battery voltage reset after 300 trigger cycles and DeadTime.
			Fixed self test 6 limits.
	002.70 - Fixed LiPO limits per WolfDragon.  Mar. 14, 2008  4:28AM
	002.80 - Set temp limit to 55C.  Set Max battery voltage to 16V.
			Added maximum Current and Temperature logging.
			Added Semi-only, Auto Roll, Soft Start, and hard battery settings to definitions to make special programming easy.
			Changed RDSon to 2818 uOhms.
			Rollover counter adjusted to 162 from 167 counts.
			Fixed high to low battery voltage lockout problem - Thanks Jawz :)
			Break time changed to 75mS.
			April 5, 2008  4:00PM
	003.00 - Rolling to 3.00 rev now :)
			Adding over temp lockout!  - Thanks Jawz!!
			Limited event log to 250 events.
			Recalibrated temperature.
			March 8, 2008  8:57PM
	003.10 - Added "Secret Mode" programming for custom chip functions through trigger pulls:
				SemiOnly Soft and hard locks.
				3-Round to full auto rollover enable / disable.
				Soft start enable / disable.
				Hard battery voltage limits.  April 11, 2008 6:46AM
	003.20 - Added low battery voltage failure warning.
			Fixed low voltage program mode problem.  April 30, 2008 10:52PM


Known issues:
	To use pin 1 (AUX1) the Reset Disable Fuse has to be set and the Debug Wire Enable Fuse has to be clear.
		( RSTDISBL = 1  DWEN = 0 )  They use pin 1 by defalt and will keep the pin high if not fixed.

	Set BrownOut voltage to 4.3V in fuses.

	Version 3.10 runs it low on memeory.  Secret Mode programming is sacrificial.

Future changes:
	None

*/


/*	Include libraries */
#include <avr/io.h> /* For all the odd ATTINY stuff */
// KDevelop workaround
#if !defined (__AVR_ATtiny85__)
  #include <avr/iotn85.h>
#endif

/*	Variable definitions */

#define VersionNumber 0x32						/* Software version number programed to EEPROM */

#define TemperatureLimit 60						/* In Celcius */
#define BreakTime 75							/* mS */

#define Neutral 0x29							/* Port control values Modified for LiPO */
#define Break 0x28
#define Motor 0x2B

/*	Initialize constants */
static const uint16_t TimeFactor = 100;					/* Adjust for 1mS step.  Magically, the calibration factor = 100 */
static const uint16_t TimeFactorFast = 1;				/* A sub mS timer counter */
static const uint16_t RDSon = 2818;				/* In uOhms (old 2870, 2766)*/
static const uint16_t HighBatteryVoltage = 16000;		/* In mV */
static const uint16_t LowBatteryVoltage = 7000;			/* In mV */
static const uint16_t TriggerHoldVoltage = 3000;		/* In mV */
static const uint16_t TrigTrip = 500;					/* In mV */
static const uint8_t TriggerFiringVoltagePercent = 80;	/* Percent */
static const uint8_t BatteryVoltageDroopWarningPercent = 85;	/* Percent */
static const uint8_t DrainCurrentPeakLimit = 250;				/* In Amps */


typedef enum
{
 Normal = 1,		/* These are GunModes made easy to read */
 ThreeRound = 2,
 ThreeRoundAuto = 2,
 SemiOnly = 3		/* Semi Only gun function */
} GunMode_t;

typedef enum 
{
	ADC20V = 0,		/* Set ADC to 20000 mV range */
	ADC10V = 1,		/* Set ADC to 10230 mV range */
	ADC1V1 = 2		/* Set ADC to 1100 mV range */
} ADCRange_t;

/*	Define ATTINY85 ports (PBx) to the gun controls */
/*	Note that these port numbers or NOT the physical pin numbers! */
typedef enum
{
	Drain = 2,			/* FET drain voltage port */
	Aux0 = 3,			/* AUX 0 I/O port */
	Trigger = 4,		/* Trigger voltage port */
	Aux1 = 5,			/* AUX 0 I/O port */
	Ground = 10,  		/* Internal A/D ground used for calibration */
	Temperature = 11	/* Internal temperature probe.  Only use in mode 2 */
} Sensor_t;

/* Function prototypes */
void MotorOFF(void);						/* Turns the motor off */
void MotorON(void);							/* Turns the motor on */
void BreakOFF(void);						/* Turns the break off */
void BreakON(void);							/* Turns the break on */
void EnergizeFETs(void);					/* Enables the FETs in a controled way */
void Timer(int32_t Time);					/* A mS timer  Timer(123) = 123mS */
void FastTimer(int32_t Time);				/* An unclaibrated ~15uS fast timer */
uint16_t GetVoltage(Sensor_t Sensor, ADCRange_t ADCRange);	/* Gets the Voltage (in mV) from the chosen source (Drain, Trigger, Aux) */
void Vibrate(void);							/* Vibrates the gun's motor for signaling the user */
void VibrateN (uint8_t VibCount);			/*  Multiple Vibrations */
void CalibrateAtoD(void);					/* Calibrates A/D convertor */
void EEPROM_write(uint8_t ucAdress, uint8_t ucData);	/* Write to EEPROM */
uint8_t EEPROM_read(uint8_t ucAddress);					/* Read from EEPROM */
void ProgramEEPROM(void);					/*  User programming of the EEPROM */
void ReadEEPROM(void);						/*  Get stored EEPROM data */
void SetBurstTime(int32_t ShotCounter);		/*	Set burst time function */
void MotorPWM(int8_t PWMduty);				/*	Motor PWM function */
void SetAUXPorts(void);						/*  Setup AUX ports */
void FactoryDiagnostics(void);				/*  Factory test */
void EventLog(int8_t);						/*  Event logging */
uint8_t LiPOStatus(void);					/*  LiPO battery check */
//	void Diagnostic(int32_t DiagTime);		/*  Diagnostic test routein */
void SecretMode (void);						/*  Secret Programming Mode */


uint8_t SemiOnlyLock = 0;				/* Semi-Only lock flag 0=off (default) 1=on 2=permant on*/
uint8_t AutoRoll = 1;					/* 3-Round auto rollover flag 0=off 1=on (default) */
uint8_t SoftStartEnable = 1;			/* Soft Start Enable 1=default */

int32_t HardBatteryVoltage = 0;				/* Hard Battery Voltage Setting in mV.  0=off*/
int32_t InitialBatteryVoltage = 0;			/* mV */
int32_t DrainCurrent = 0;				/* The current through the drive FET in Amps */
int32_t VoltageOffset0 = 0;				/* A/D 5.0V REF offest error */
int32_t VoltageOffset1 = 0;				/* A/D 2.56V REF offest error */
int32_t VoltageOffset2 = 0;				/* A/D 1.1V REF offest error */
int32_t SBTCounter = 0;					/* Burst time function counter */

uint8_t BurstTimeFactor = 80;			/* The single to three round burst ratio DEFAULT if EEPROM = FF*/
										/* 100 = 3.00 X the single shot time. 2X+100% */
int16_t BurstTimeFactorPercent = 0;		/* Real BurstTimeFactorPercent */
int16_t SingleShotTime = 70;			/* Default loops for single shot */
int16_t BurstTime = 205;				/* Default motor on time for a burst */
int16_t SingleShotTimeOld = 0;			/* Old value of single shot time */
uint8_t MotorSpeed = 254;				/* 0 to 254 PWM value */
uint8_t ErrorCode = 0;					/* The Errors are each given a unique number */
uint8_t GunMode = Normal;				/* 1=trigger control (Normal) 2=burst (ThreeRoundAuto) */
uint8_t TriggerHold = 0;				/* Hold further shotting until trigger is released */
uint8_t GunFunction = Normal;			/* Operating mode DEFAULT if EEPROM = FF */
										/* 1 = Normal  2 = ThreeRound  3 = Semi Only*/
int8_t  GunFunctionFlag = 0;			/* Counter used in mode programing */
uint8_t LiPOInstalled = 0;			/* LiPO Installed flag */
uint8_t TemperatureMAX = 0;			/* Maximum temperature seen */
uint8_t CurrentMAX = 0;				/* Maxiumum current seen / 10 */

__attribute__ ((noreturn))
void main(void)
{
	uint32_t FiringCounter = 0;		/* Counts firing cycles*/
	int32_t TriggerVoltage = 0;		/* The voltage on the trigger input in mV */
	int32_t ModeCounter = 0;		/* A main loop counter for the threeround auto fuction */
	int32_t ShotCounter = 0;		/* Shot count inside a burst */
	int32_t DeadTimeCounter = 0;	/* Counter used to find battery rest time */
	int32_t BatteryVoltageDroopWarning = 5500;	/* mV */
	int32_t TriggerFiringVoltage = 6000;		/* mV */
	int16_t RunningTemp = 0;		/* CPU temperature */
	uint8_t DroopWarningFlag = 0;	/* Droop warning counter */
	uint8_t DroopWarningFlag1 = 0;	/* Hard Droop warning counter */
	uint8_t OverTempLockout = 0;	/* Over temperature lockout flag */
	uint8_t LiPO = 0;				/* LiPO battery status */
	uint8_t BatteryLimitReset = 0;		/* LiPO Installed flag */
    /*	MAIN POWER UP */

    /*	Wait 2 seconds for hardware to stabilize */
    Timer(2000);
    ErrorCode = 1;
    GunMode = Normal;

    /*	Setup A/D parameters */
    CalibrateAtoD();

    /*	Check trigger voltage */
    if (GetVoltage(Trigger, ADC20V) > TrigTrip)
    {
        ErrorCode = 4;
    }

    /*  Get EEPROM data */
    ReadEEPROM();

    /*	Energize drive circuits */
    EnergizeFETs();

    /*  Setup AUX ports */
    SetAUXPorts();

    /*	Check drain voltage */
    uint16_t DrainVoltage = GetVoltage(Drain, ADC20V); /* The voltage on the MOSFET drains mV */
    if (DrainVoltage < LowBatteryVoltage)
    {
        ErrorCode = 2;
    }
    if (DrainVoltage > HighBatteryVoltage)
    {
        EventLog(1);
        ErrorCode = 3;
    }

    /*  Set Battery Limits */
    if (InitialBatteryVoltage == 0)
    {
        InitialBatteryVoltage = (GetVoltage(Drain, ADC20V));
    }
    if (HardBatteryVoltage != 0)
    {
        InitialBatteryVoltage = HardBatteryVoltage;
    }
    TriggerFiringVoltage = (InitialBatteryVoltage * TriggerFiringVoltagePercent) / 100;
    BatteryVoltageDroopWarning = (InitialBatteryVoltage * BatteryVoltageDroopWarningPercent) / 100;

    LiPO = LiPOStatus();

    /*	Vibrate status to motor */
    for (uint8_t count = 0; count < ErrorCode; count++)
    {
        Timer(500);
        Vibrate();
    }

    /*Factory Test */
    FactoryDiagnostics();

    /*  If the battery voltage is not safe, force a shutdown */
Halt:
    if (ErrorCode > 1)
    {
        goto Halt;
    }
    /*	Error Codes
    1	Normal operation
    2	Battery voltage too low
    3	Battery voltage too high
    4   Trigger voltage too high
    16	Low battery
    17  High motor peak current
    18  Overheating
    */

    /*	Wait for trigger pull to enter programming mode (2 seconds) */
    for (uint16_t count=0; count < 1185; count++)
    {
        if (GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)
        {
            ProgramEEPROM();
        }
    }

    /* Semi-Only Locked here From SemiOnlyLock definition */
    if (SemiOnlyLock != 0)
    {
        GunFunction = 3;
    }


    /*	MAIN SCANNING LOOP */
    Vibrate();
    Vibrate();
    Vibrate();
    Vibrate();
    if (LiPOInstalled != 0)
    {
        Vibrate();
        Vibrate();
        Vibrate();
        Vibrate();
    }

MainLoop:
    ModeCounter++;
    DeadTimeCounter++;
    /*	Set motor to off no matter what is going on */
    MotorOFF();

    /*	Check trigger voltage */
    TriggerVoltage = GetVoltage(Trigger, ADC20V);

    if ((TriggerVoltage < TriggerFiringVoltage) && (TriggerVoltage > LowBatteryVoltage))
    {
        DroopWarningFlag1++;
        if (DroopWarningFlag1 > 5)
        {
            Vibrate();
            Timer(1000);
            DroopWarningFlag1 = 0;
        }
    }

    if ((TriggerVoltage >= TriggerFiringVoltage) && (TriggerHold == 0))
    {

        if (LiPOInstalled == 0)
        {
            /*	Check trigger voltage incase the battery is dying. Check three times to stop false warnings */
            if (TriggerVoltage < BatteryVoltageDroopWarning)
            {
                DroopWarningFlag++;
            }
            else
            {
                DroopWarningFlag = 0;
                goto Fire;
            }
            if (DroopWarningFlag > 2)
            {
                ErrorCode = 16;
            }
            goto Fire;
        }
        else
        {
            LiPO = LiPOStatus();
            if (LiPO == 1)
            {
                DroopWarningFlag = 0;
                goto Fire;
            }
            DroopWarningFlag++;
            if ((LiPO == 3) && (DroopWarningFlag > 2))
            {
                (ErrorCode = 16);
                goto CeaseFire;
            }
            if ((LiPO == 2) && (DroopWarningFlag > 2))
            {
                ErrorCode = 16;
                goto Fire;
            }
            goto Fire;
        }
    }
    if (TriggerVoltage < TrigTrip)
    {
        TriggerHold = 0;
        GunMode = Normal;
        ModeCounter = 0;
    }

    /* Goto full auto in three round burst mode if trigger is held down 1/2 second inter */
    if ((ModeCounter > 162) && (TriggerHold == 1) && (GunFunction != 3) && (AutoRoll == 1))
    {
        GunMode = ThreeRoundAuto;
        goto Fire;
    }


    /*  Reset Battery Limits after 300 triggers*/
    if ((BatteryLimitReset == 0) && (FiringCounter > 300) && (DeadTimeCounter > 1000) && (HardBatteryVoltage != 0))
    {
        InitialBatteryVoltage = (GetVoltage(Drain, ADC20V));
        TriggerFiringVoltage = (InitialBatteryVoltage * TriggerFiringVoltagePercent) / 100;
        BatteryVoltageDroopWarning = (InitialBatteryVoltage * BatteryVoltageDroopWarningPercent) / 100;
        EventLog(4);
        BatteryLimitReset = 1;
    }

    /*	Return to top and loop again */
    goto MainLoop;


    /*	FIRING LOOP */
Fire:
    if (OverTempLockout == 1)
    {
        goto CeaseFire;
    }
    /* MOTOR IS ON!!!!! */
    MotorON();
    Timer(1);	/* A delay to avoid the motor start up electrical noise burst */
    ShotCounter = 0;

    FiringCounter++;  /* increment firing loop counter */

ShotDetect:  /* Main firing loop. */
    ShotCounter++;
    TriggerVoltage = GetVoltage(Trigger, ADC10V);	/* Get trigger voltage */
    if (TriggerVoltage < TriggerHoldVoltage)  /* Is trigger off now? */
    {
        if ((GunFunction == Normal) || (GunMode == ThreeRoundAuto))
        {
            GunMode = Normal;    /* normally just CeaseFire */
            goto CeaseFire;
        }
        if (TriggerHold == 0)
        {
            goto CeaseFire;
        }
    }
    if ((GunMode == ThreeRoundAuto) || (GunFunction == Normal))
    {
        goto ShotDetect;   /* Keep cycling in auto modes */
    }
    if (ShotCounter > BurstTime)
    {
        TriggerHold = 1;    /* Three round time delay wait */
        goto CeaseFire;
    }
    goto ShotDetect;


    /*	FIRING HALT */

CeaseFire:

    /*	The motor ON and OFF functions have a 1mS anti cross conduction safety delay built in */
    /*	Turn motor to off */
    MotorOFF();

    /*	Turn break to on */
    BreakON();

    /*	Leave the break on for some time in mS */
    Timer(BreakTime);

    /*	Turn break to off */
    BreakOFF();
    /*  If in semi mode, set burst time */
    if (TriggerHold == 0)
    {
        SetBurstTime(ShotCounter);
    }

    /*  Check unit temperature */
    RunningTemp = GetVoltage(Temperature, ADC1V1);

    RunningTemp = (RunningTemp - 298) * 100 / 109;
    if (RunningTemp >= TemperatureLimit)
    {
        EventLog(3);
        OverTempLockout = 1;
    }
    if (RunningTemp <= (TemperatureLimit - 5))
    {
        OverTempLockout = 0;
    }
    if (OverTempLockout == 1)
    {
        ErrorCode = 18;
    }


    ModeCounter = 0;
    DeadTimeCounter = 0;

    /*Maximum Temperature */
    if (RunningTemp > TemperatureMAX)
    {
        TemperatureMAX = RunningTemp;
        EEPROM_write(0x10, (char)TemperatureMAX);
    }

    /*Maximum Current */
    if (DrainCurrent / 10 > CurrentMAX)
    {
        CurrentMAX = DrainCurrent / 10;
        EEPROM_write(0x11, (char)CurrentMAX);
    }

    /*  If an error occured report it */
    if (ErrorCode == 0)
    {
        goto MainLoop;
    }
    /*	16 - Low battery - Vibrate motor once after firing
    	17 - High motor peak current - Vibrate motor twice
    	18 - Overheating - Vibrate motor three times
    */
    Timer(100);
    if (ErrorCode == 16)
    {
        VibrateN(1);
    }
    if (ErrorCode == 17)
    {
        VibrateN(2);
    }
    if (ErrorCode == 18)
    {
        VibrateN(3);
    }

    ErrorCode = 0;
    goto MainLoop;

}  /* End of main */



/* FUNCTIONS */

/* For safety in that "nothing can go wrong", the motor drive ports are controlled directly */
void MotorOFF(void)
{
    MotorPWM(0);
}


void MotorON(void)
{
    PORTB = Neutral; 	/* Be REALLY(!!) sure break is off */
    Timer(1);		/* Wait 1mS for anti-cross conduction */
    PORTB = Motor;	/* motor is on for peak current test */
    Timer(1);	/* Wait to stabalize */
    /*	Check drain current for over peak current limit */
    DrainCurrent = GetVoltage(Drain, ADC10V) * 1000 / RDSon;
    PORTB = Neutral;	/* motor is off after test */
    if (DrainCurrent > DrainCurrentPeakLimit)
    {
        PORTB = Neutral;    /* Shutdown if overcurrent */
        EventLog(2);
        ErrorCode = 17;
        return;
    }

    MotorPWM(1);
    /* Soft Start */
    if (SoftStartEnable == 1)
    {
        for (uint8_t counter = 0; counter < 51; counter++)
        {
            uint8_t RampSpeed = MotorSpeed / 2 + (MotorSpeed * counter) / 100;
            OCR0B = (char)RampSpeed;
        }
    }
    else
    {
        OCR0B = (char)MotorSpeed;
    }
}


void BreakOFF(void)
{
    PORTB = Neutral;   /* The break signal is negative logic */
}


void BreakON(void)
{
    PORTB = Neutral;	/* Be REALLY(!!) sure motor is off */
    Timer(1);		/* Wait 1mS for anti-cross conduction */
    PORTB = Break;	/* The break signal is negative logic */
}


void Timer(int32_t Time)  /* Timer(100) = 100mS */
{
    Time = Time * TimeFactor;
    for (int32_t counter = 0; counter <= Time; counter++) {}
}


void FastTimer(int32_t Time) /* Timer(50) = 672uS */
{
    Time = Time * TimeFactorFast;
    for (int32_t counter = 0; counter <= Time; counter++) {}
}


void EnergizeFETs(void)
{
    /*	Energize break circuit */
    /*	Break is on PB0 */
    PORTB = 0x00;	/* Set pullups off */
    DDRB = 0x01;	/* Set port B pin 0 for output */
    PORTB = Neutral;	/* Set port B pin 0 to 1 */

    /*	Wait 1000mS for capacitive break circuit to stabilize (very important!!) */
    Timer(1000);

    /*	Energize motor circuit */
    /*	Motor is on PB1 */
    DDRB = 0x03;	/* Set port B pins 0 and 1 for output */
    PORTB = Neutral;	/* Set port B pin 0 to 1 and pin 1 to 0 */
    /*	Wait 1mS */
    Timer(1);
}


void SetAUXPorts(void)
{
    /*  Set AUX ports */
    /* Turn on pullups for Aux0 and Aux1. Set motor off and break off */
    /* Pullups are set in the Nuetral, Break, and Motor definitions. */
    DDRB = 0x03;
    PORTB = Neutral;
}


uint16_t GetVoltage(Sensor_t Sensor, ADCRange_t ADCRange)
{
    int32_t  Voltage = 0;				/* A/D convertor returned voltage in mV */
    if (ADCRange == ADC1V1)				/* 0-1.1V range */
    {
		ADMUX = 0x80;
	}
	else if (ADCRange == ADC10V)		/* 0-10.23V range */
    {
		ADMUX = 0x90;
	}
    else /*if (ADCRange == ADC20V)*/	/* 0-20V range */
	{
		ADMUX = 0x00;
	}
	
	switch (Sensor)
	{
		case Aux1:
			ADMUX |= 0x00;
			break;
		case Drain:
			ADMUX |= 0x01;
			break;
		case Trigger:
			ADMUX |= 0x02;
			break;
		case Aux0:
			ADMUX |= 0x03;
			break;
		case Ground:
			ADMUX |= 0x0D;   /* Internal ground for calibration */
			break;
		case Temperature:
			if (ADCRange == ADC1V1)
			{
				ADMUX = 0x0F;   /* Can only be used with 1.1V referance */
				break;
			}
		default:
			ADMUX |= 0x0D;
	}

    /* Start A/D conversion */
    ADCSRA = 0b11000110;  						/* Start A/D conversion (bit 6 is set) 125kHz clock*/
    /* Test if done (bit 6 goes clear, but is set when A/D is complete) */
    do {}
    while (bit_is_set(ADCSRA,6));
    /* Get voltage Vref = 5000mV  Rdivider = 1/4  Full range is 20V */
    Voltage = ADCW;								/* This word is the 10 bit A/D result */
    if (ADCRange == ADC20V)
    {
        Voltage = Voltage - VoltageOffset0;  	/* A/D Offset error adjustment */
        if (Voltage < 0)
        {
            Voltage = 0;
        }
        Voltage = ((Voltage * 20000) / 1023);	/* Convert A/D full range (1023) to 19980mV or 19.98V */
    }
    if (ADCRange == ADC10V)
    {
        Voltage = Voltage - VoltageOffset1;  	/* A/D Offset error adjustment */
        if (Voltage < 0)
        {
            Voltage = 0;
        }
        Voltage = Voltage * 10;					/* Convert A/D full range (1023) to 10230mV or 10.23V */
    }
    if (ADCRange == ADC1V1)
    {
        Voltage = Voltage - VoltageOffset2;  	/* A/D Offset error adjustment */
        if (Voltage < 0)
        {
            Voltage = 0;
        }
        Voltage = Voltage * 1100 / 1023;		/* Convert A/D full range (1023) to 1100mV or 1.100V */
    }
    return (uint16_t)Voltage;
}


void CalibrateAtoD(void)
{
    /* 5.0V REF  Range = 20V */
    VoltageOffset0 = 0;							/* Clear old value */
    VoltageOffset0 = GetVoltage (Ground, ADC20V);	/* Run the converter with grounded input */
    VoltageOffset0 = ADCW;
    /* 2.56V REF  Range = 10.23V */
    VoltageOffset1 = 0;							/* Clear old value */
    VoltageOffset1 = GetVoltage (Ground, ADC10V);	/* Run the converter with grounded input */
    VoltageOffset1 = ADCW;						/* Get the raw A/D converter output word */
    /* 1.1V REF  Range = 1.1V */
    VoltageOffset2 = 0;							/* Clear old value */
    VoltageOffset2 = GetVoltage (Ground, ADC1V1);	/* Run the converter with grounded input */
    VoltageOffset2 = ADCW;						/* Get the raw A/D converter output word */
}


void Vibrate(void)
{
    /* Vibrate for 1/2 second */
    BreakOFF();
    Timer(1);
    for (uint8_t counter = 0; counter < 10; counter++)
    {
        PORTB = Motor;
        FastTimer(20);		/* Wait 500uS for motor on pulse */
        PORTB = Neutral;
        Timer(25);
    }
}

void VibrateN(uint8_t VibCount)
/* Vibrate multiple times */
{
    for (uint8_t counter = 0; counter < VibCount; counter++)
    {
        Vibrate();
        Timer(250);
    }
}


/* EEPROM stuff copied directly from ATMEL data sheet */

void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
    /* Wait for completion of previous write */
    while (EECR & (1<<EEPE));
    /* Set programming mode */
    EECR = (0<<EEPM1) | (0<<EEPM0);
    /* Set up address and data registers */
    EEAR = ucAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}


uint8_t EEPROM_read(uint8_t ucAddress)
{
    /* Wait for completion of previous write */
    while (EECR & (1<<EEPE));
    /* Set up address register */
    EEAR = ucAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from data register */
    return EEDR;
}


void ProgramEEPROM(void)
{

    /* Change EEPROM Programming */
    /*
    0x00 = Version Number
    0x01 = GunFunction
    0x02 = BurstTimeFactor
    0x03 = SingleShotTime
    0x04 = Motor Speed
    0x05 = Initial Battery Voltage / 100
    0x10 = Maximum Temperature
    0x11 = Maximum Current / 10
    0x20 = SemiOnlyLock
    0x21 = AutoRoll
    0x22 = SoftStartEnable
    0x32 = HardBatteryVoltage / 100
    */

    Timer(500);

    Vibrate();
    /* Pull Trigger now to go to set GunFunction by number of trigger pulls.  Limit = 2 */
    GunFunctionFlag = 0;
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;   /* Tigger hold is a debounce for the trigger switch */
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
        {
            GunFunctionFlag++;
            GunFunction = GunFunctionFlag;
            if (GunFunction > 3)
            {
                GunFunction = 3;   /* limit is mode 3 */
            }
            count = 0;
            TriggerHold = 1;
        }
    }

    VibrateN(1);
    Vibrate();
    /* Pull Trigger now to go to decrease BurstTimeFactor by number of trigger pulls.  Limit = 0*/
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;   /* Tigger hold is a debounce for the trigger switch */
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage) && (TriggerHold == 0) && (BurstTimeFactor > 0)))
        {
            BurstTimeFactor--;
            count = 0;
            TriggerHold = 1;
        }
    }

    VibrateN(2);
    Vibrate();
    /* Pull Trigger now to go to increase BurstTimeFactor by number of trigger pulls.  Limit = 250 */
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;   /* TiggerHold is a debounce for the trigger switch */
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage) && (TriggerHold == 0) && (BurstTimeFactor < 250)))
        {
            BurstTimeFactor++;
            count = 0;
            TriggerHold = 1;
        }
    }

    VibrateN(3);
    Vibrate();
    /* Pull Trigger now to go to decrease MotorSpeed by number of trigger pulls.  Limit = 52*/
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;   /* Tigger hold is a debounce for the trigger switch */
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage) && (TriggerHold == 0) && (MotorSpeed > 0)))
        {
            MotorSpeed = (MotorSpeed * 9) / 10;
            count = 0;
            SBTCounter = 0;
            TriggerHold = 1;
        }
    }

    VibrateN(4);
    Vibrate();
    /* Pull Trigger now to go to increase MotorSpeed by number of trigger pulls.  Limit = 254 */
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;   /* TiggerHold is a debounce for the trigger switch */
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage) && (TriggerHold == 0) && (MotorSpeed < 255)))
        {
            if (MotorSpeed < 232)
            {
                MotorSpeed = (MotorSpeed * 11) / 10;
                count = 0;
                SBTCounter = 0;
                TriggerHold = 1;
            }
            else
            {
                MotorSpeed = 254;
                count = 0;
                SBTCounter = 0;
                TriggerHold = 1;
            }
        }
    }

    /* Store EEPROM data */
    /* Gun Function */
    EEPROM_write (0x01, (char)GunFunction);
    /* Burst time factor */
    if (BurstTimeFactor < 50)
    {
        BurstTimeFactor = 50;
    }
    if (BurstTimeFactor >250)
    {
        BurstTimeFactor = 250;
    }
    EEPROM_write(0x02, (char)BurstTimeFactor);
    /* MotorSpeed */
    if (MotorSpeed < 52)
    {
        MotorSpeed = 52;
    }
    if (MotorSpeed >254)
    {
        MotorSpeed = 254;
    }
    EEPROM_write(0x04, (char)MotorSpeed);
    InitialBatteryVoltage = (GetVoltage(Drain, ADC20V));
	uint8_t BatteryVoltageEEPROM = 0;	/* Stored initial battery voltage */
    BatteryVoltageEEPROM = (char)(InitialBatteryVoltage / 100);
    EEPROM_write(0x05, (char)BatteryVoltageEEPROM);

    VibrateN(5);
    Vibrate();
    /* Pull Trigger now to go to Master Reset */
    uint16_t holdCounter = 0;
    for (uint16_t count=0; count < 600; count++)
    {
        while (GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)
        {
            holdCounter++;
            if (holdCounter > 1500)
            {
                EEPROM_write(0x00,0xFF);
                EEPROM_write(0x01,0xFF);
                EEPROM_write(0x02,0xFF);
                EEPROM_write(0x03,0xFF);
                EEPROM_write(0x04,0xFF);
                EEPROM_write(0x05,0xFF);

                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();
                Vibrate();



                /* Pull Trigger now 10 times to go to secret mode */
                uint8_t pullCount = 0;
                for (uint16_t count=0; count < 600; count++)
                {
                    if (GetVoltage(Trigger, ADC20V) < TrigTrip)
                    {
                        TriggerHold = 0;
                    }
                    if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
                    {
                        pullCount++;
                        if (pullCount == 10)
                        {
                            SecretMode();
                        }
                        count = 0;
                        TriggerHold = 1;
                    }
                }

Halt1:
                goto Halt1;
            }
        }
    }

}


void ReadEEPROM(void)
{
    /* Note - Erased EEPROM = FF not 00 */
    /*
    	0x00 = Version Number
    	0x01 = GunFunction
    	0x02 = BurstTimeFactor
    	0x03 = SingleShotTime
    	0x04 = Motor Speed
    	0x05 = Initial Battery Voltage / 100
    	0x10 = Maximum Temperature
    	0x11 = Maximum Current / 10
    	0x20 = SemiOnlyLock
    	0x21 = AutoRoll
    	0x22 = SoftStartEnable
    	0x32 = HardBatteryVoltage / 100
    */
    /* Version */
    EEPROM_write(0x00, (char)VersionNumber);

    /* Gun Function */
    if (EEPROM_read(0x01) != 0xFF)
    {
        GunFunction = EEPROM_read(0x01);
    }

    /* Burst time factor */
    if (EEPROM_read(0x02) != 0xFF)
    {
        BurstTimeFactor = EEPROM_read(0x02);
    }

    /* Single Shot Time */
    if (EEPROM_read(0x03) != 0xFF)
    {
        SingleShotTime = EEPROM_read(0x03);
        SBTCounter = 0;

        BurstTimeFactorPercent = 2 * BurstTimeFactor + 100;
        BurstTime = (BurstTimeFactorPercent * 117 * SingleShotTime) / 120 / 100 + (37 * (BurstTimeFactorPercent - 100) / 120);


        if (GunFunction == 3)
        {
            BurstTime = SingleShotTime;   /* Semi only mode */
        }
    }

    /* Motor Speed */
    MotorSpeed = EEPROM_read(0x04);

    /*  0x05 = Initial Battery Voltage / 100  */
    if (EEPROM_read(0x05) != 0xFF)
    {
        InitialBatteryVoltage = EEPROM_read(0x05) * 100;
    }

    /* Maximum Temperature */
    if (EEPROM_read(0x10) != 0xFF)
    {
        TemperatureMAX = EEPROM_read(0x10);
    }

    /* Maximum Current */
    if (EEPROM_read(0x11) != 0xFF)
    {
        CurrentMAX = EEPROM_read(0x11);
    }

    /* Lock Flags */
    if (EEPROM_read(0x20) != 0xFF)
    {
        SemiOnlyLock = EEPROM_read(0x20);
    }
    if (EEPROM_read(0x21) != 0xFF)
    {
        AutoRoll = EEPROM_read(0x21);
    }
    if (EEPROM_read(0x22) != 0xFF)
    {
        SoftStartEnable = EEPROM_read(0x22);
    }
    if (EEPROM_read(0x23) != 0xFF)
    {
        HardBatteryVoltage = (EEPROM_read(0x23)) * 100;
    }
}


void SetBurstTime(int32_t ShotCounter)
{
    SBTCounter++;
    /* Shot Three after gun is powered up is the golden shot */
    if (SBTCounter == 3)
    {
        SingleShotTime = ShotCounter;
    }
    /* Do not update if ShotCounter is < 90% or > 110% of SingleShotTime (early user trigger release) */
    if (SBTCounter > 3)
    {
        if ((ShotCounter > (SingleShotTime * 11) / 10) || (ShotCounter < (SingleShotTime * 9) / 10))
        {
            return;
        }
    }
    else
    {
        SingleShotTime = ((SingleShotTime * 5) + ShotCounter) / (5 + 1);
    }
    /*
    	SingleShotTime = 1.17 x ShotCounter + 37 in mS.
    	BurstTime = 1.20 x ShotCounter + 37 in mS.
    	BurstTime = BurstTimeFactor x 1.17 / 1.20 x SingleShotTime + 37 / 1.20 x (BurstTimeFactor - 1)
    */
    BurstTimeFactorPercent = 2 * BurstTimeFactor + 100;
    BurstTime = (BurstTimeFactorPercent * 117 * SingleShotTime) / 120 / 100 + (37 * (BurstTimeFactorPercent - 100) / 120);

    if (BurstTime < SingleShotTime)
    {
        BurstTime = SingleShotTime;   /* Be sure a BurstTime error will not lock out the gun */
    }
    /* Store SingleShotTime now */
    if (SingleShotTime < 0)
    {
        SingleShotTime = 1;
    }
    if (SingleShotTime >254)
    {
        SingleShotTime = 254;
    }
    if (SingleShotTime != SingleShotTimeOld)
    {
        EEPROM_write(0x03, (char)SingleShotTime);
        SingleShotTimeOld = SingleShotTime;
    }
    /* Semi only mode */
    if (SBTCounter < 3)
    {
        BurstTime = 2 * SingleShotTime;
    }
    else
    {
        if (GunFunction == 3)
        {
            BurstTime = SingleShotTime - 2;
        }
    }
}


void MotorPWM(int8_t PWMduty)
/* Controlling OC0B which is pin 6 to the motor drive */
{
    if (PWMduty == 0)
    {
        TCCR0A = 0x03;    /* Shutdown */
        GTCCR = 0x80;
        PORTB = Neutral;
        return;
    }
    GTCCR = 0x81; 				/* HALT, SYNC, and RESET the PWM counter */
    TCCR0A = 0x23;				/* Enable PWMcontrol of motor and set function */
    TCCR0B = 0x01;				/* Divide by 8 for ~4000hZ */
    TCNT0 = (char)PWMduty;		/* Zero the counter */
    OCR0B = (char)PWMduty;		/* This is the PWM comparator test value */
    TIMSK = 0x08;				/* Be sure the harware interupts are off but compare OS0B */
    GTCCR = 0x00;				/* Start the PWM */
}


void FactoryDiagnostics(void)
/* Factory test */
{
	int32_t TestValue = 0;		/* Factory test variable */
	uint8_t TestCode = 0;		/* Facotry test variable */
    /* Test for Trigger Key voltage */
    TestValue = GetVoltage(Trigger, ADC20V);
    if ((TestValue < 4392) || (TestValue > 5368))
    {
        return;
    }
    EventLog(0);
    /* Turn on data ports for output - Only reset when battery disconnected */
    DDRB = 0x2B;
    PORTB = 0x01;
TestResult:
    TestCode = 1;
    PORTB = 0x01; /* Turn FETs off */
    Timer(2000);
    TestValue = GetVoltage(Trigger, ADC20V); /* Trigger sense test */
    if ((TestValue < 4734) || (TestValue > 5025))
    {
        TestCode = 2;
    }
    TestValue = GetVoltage(Trigger, ADC10V); /* Trigger sense test */
    if ((TestValue < 4734) || (TestValue > 5025))
    {
        TestCode = 3;
    }
    TestValue = GetVoltage(Drain, ADC10V); /* Drain sense test */
    if ((TestValue < 4734) || (TestValue > 5025))
    {
        TestCode = 4;
    }
    PORTB = 0x03; /* Turn Q1 Drive FET on */
    Timer(1);
    TestValue = GetVoltage(Drain, ADC10V); /* Q1 test */
    if ((TestValue < 0) || (TestValue > 10))
    {
        TestCode = 5;
    }
    PORTB = 0x01; /* Turn Q1 Drive FET off */
    Timer(1);
    PORTB = 0x00; /* Turn Q2 Break FET on */
    Timer(1);
    TestValue = GetVoltage(Drain, ADC10V); /* Q2 test */
    if ((TestValue < 9701) || (TestValue > 10301))
    {
        TestCode = 6;
    }
    PORTB = 0x01; /* Turn Q2 Break FET off */
    Timer(1);
    /* Blink Error Code to AUX5 LED */
    for (uint8_t TestCounter = 0; TestCounter < TestCode; TestCounter++)
    {
        PORTB = 0x21; /* LED on */
        Timer(500);	/* Wait 500mS */
        PORTB = 0x01; /* LED off */
        Timer(500);	/* Wait 500mS */
    }
    if (TestCode != 1)
    {
        /* Output bad value visually in mS */
        Timer(1500);
        PORTB = 0x21; /* LED on */
        Timer(TestValue);
        PORTB = 0x01; /* LED off */
        Timer(2000);
    }

    /*
    1 = Normal
    2 = 20V Trigger Voltage
    3 = 10V Trigger Voltage
    4 = Drain open circuit voltage
    5 = Q1 FET drive
    6 = Q2 FET drive
    */

    /* Store Lock flags to EEPROM */
    EEPROM_write(0x20, (char)SemiOnlyLock);
    EEPROM_write(0x21, (char)AutoRoll);
    EEPROM_write(0x22, (char)SoftStartEnable);
    EEPROM_write(0x23, (char)HardBatteryVoltage / 100);

    goto TestResult; /* Keep Flashing Code */

}

void EventLog(int8_t Event)
/*	0 = Factory Test Performed	0x40
	1 = High Battery Voltage  	0x41
	2 = Peak Current Limit		0x42
	3 = Over Temperature Limit	0x43
	4 = Battery Voltage Reset	0x44 */
{
	uint8_t EventOld = 0;	/* Event logging variable */
    Event = Event + 64;
    EventOld = EEPROM_read((char)(Event));
    EventOld++;
    if (EventOld > 250)
    {
        EventOld = 250;
    }
    EEPROM_write((char)Event,(char)EventOld);
    return;
}


uint8_t LiPOStatus(void)
/*	0 = No Monitor Present
	1 = LiPO Battery OK
	2 = LiPO Battery Weak
	3 = LiPO Battery Dead  */
{
    PORTB = Neutral;	/* Be sure pullups are on */
    Timer(1);
    /* Aux voltages are /4 since there are no voltage dividers */
    /* Voltages are im mV */
    uint16_t Aux0Data = GetVoltage(Aux0, ADC20V) / 4;
    uint16_t Aux1Data = GetVoltage(Aux1, ADC20V) / 4;
    if ((Aux0Data < 300) && (Aux1Data < 300))
    {
        LiPOInstalled = 1;
        return 1;
    }
    if (Aux1Data > 500)
    {
        return 3;
    }
    if (Aux0Data > 500)
    {
        return 2;
    }
    return 1;
}


//void Diagnostic(int32_t DiagTime)
/* program test output to led.  Only for development use */
/*
	{
	DDRB = 0x2B;
	PORTB = 0x21; 		// LED on
	Timer(RunningTemp);	// Wait 500mS
	PORTB = 0x01; 		// LED off
	Timer(500);			// Wait 500mS
	SetAUXPorts();
	}
*/

void SecretMode(void)
{
    /* Semi Lock */
    Vibrate();
    if (EEPROM_read(0x20) != 0x02)
    {
        EEPROM_write(0x20, 0x00);
    }
	uint8_t holdCount = 0;
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
        {
            if (EEPROM_read(0x20) == 0x00)
            {
                EEPROM_write(0x20, 0x01);
            }
            holdCount++;
            if (holdCount == 10)
            {
                EEPROM_write(0x20, 0x02);
            }
            count = 0;
            TriggerHold = 1;
        }
    }

    /* AutoRoll */
    VibrateN(1);
    Vibrate();
    EEPROM_write(0x21, 0x01);
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
        {
            EEPROM_write(0x21, 0x00);
            count = 0;
            TriggerHold = 1;
        }
    }

    /* Soft Start */
    VibrateN(2);
    Vibrate();
    EEPROM_write(0x22, 0x01);
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
        {
            EEPROM_write(0x22, 0x00);
            count = 0;
            TriggerHold = 1;
        }
    }

    /* HardBatteryVoltage */
    VibrateN(3);
    Vibrate();
    EEPROM_write(0x23, 0x00);
    holdCount = 0;
    for (uint16_t count=0; count < 600; count++)
    {
        if (GetVoltage(Trigger, ADC20V) < TrigTrip)
        {
            TriggerHold = 0;
        }
        if (((GetVoltage(Trigger, ADC20V) > LowBatteryVoltage)) && (TriggerHold == 0))
        {
            EEPROM_write(0x23, (char)(70 + 5 * holdCount));
            holdCount++;
            count = 0;
            TriggerHold = 1;
        }
    }
}

/* End of program */
