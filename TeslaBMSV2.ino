#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h>

#include <FlexCAN.h>
#include <SPI.h>

#define CPU_REBOOT (_reboot_Teensyduino_());

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

//Simple BMS Settings//
int CAP = 100; //battery size in Ah
int Pstrings = 2; // strings in parallel used to divide voltage of pack
int ESSmode = 0; //turn on ESS mode, does not respond to key switching
float storagedelta = 0.3; //in ESS mode in 1 high changes charge and discharge limits by this amount


//Simple BMS V2 wiring//
const int ACUR1 = A0; // current 1
const int ACUR2 = A1; // current 2
const int IN1 = 16; // input 1 - high active
const int IN2 = 17; // input 2- high active
const int IN3 = 19; // input 1 - high active
const int IN4 = 18; // input 2- high active
const int OUT1 = 11;// output 1 - high active
const int OUT2 = 12;// output 1 - high active
const int OUT3 = 20;// output 1 - high active
const int OUT4 = 21;// output 1 - high active
const int OUT5 = 22;// output 1 - high active
const int OUT6 = 23;// output 1 - high active
const int OUT7 = 5;// output 1 - high active
const int OUT8 = 6;// output 1 - high active
const int led = 13;
const int BMBfault = 11;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define Error 5
//
int cursens = 2;
//Current sensor values
#define Undefined 0
#define Analogue 1
#define Canbus 2
//


int Discharge;

//variables for output control
int pulltime = 1000;
int contctrl, contstat = 0; //1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer, Pretimer = 0;
int Pretime = 5000; //precharge timer
int conthold = 50; //holding duty cycle for contactor 0-255
int Precurrent = 1000; //ma before closing main contator

int gaugelow = 255; //empty fuel gauge pwm
int gaugehigh = 70; //full fuel gauge pwm

//variables for VE driect bus comms
char* myStrings[] = {"V", "14674", "I", "0", "CE", "-1", "SOC", "800", "TTG", "-1", "Alarm", "OFF", "Relay", "OFF", "AR", "0", "BMV", "600S", "FW", "212", "H1", "-3", "H2", "-3", "H3", "0", "H4", "0", "H5", "0", "H6", "-7", "H7", "13180", "H8", "14774", "H9", "137", "H10", "0", "H11", "0", "H12", "0"};

//variables for VE can
uint16_t chargevoltage = 49100; //max charge voltage in mv
int chargecurrent, chargecurrentmax = 300; //max charge current in 0.1A
uint16_t disvoltage = 42000; // max discharge voltage in mv
int discurrent, discurrentmax = 300; // max discharge current in 0.1A
int batvcal = 0;

uint16_t SOH = 100; // SOH place holder

unsigned char alarm[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'S', 'I', 'M', 'P', ' ', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'T', 'O', 'M', ' ', 'D', 'E', ' ', 'B'};
long unsigned int rxId;
unsigned char len = 0;
byte rxBuf[8];
char msgString[128];                        // Array to store serial string
uint32_t inbox;
signed long CANmilliamps;

//struct can_frame canMsg;
//MCP2515 CAN1(10); //set CS pin for can controlelr


//variables for current calulation
int value;
int invertcur = 0;
uint16_t offset1 = 1735;
uint16_t offset2 = 1733;
int highconv = 285;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long looptime = 0; //ms
int currentsense = 14;
int sensor = 1;

//running average
const int RunningAverageCount = 16;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
uint16_t socvolt[4] = {3100, 10, 4100, 90};


//variables
int outputstate = 0;
int incomingByte = 0;
int x = 0;
int storagemode =0;

//Debugging modes//////////////////
int debug = 1;
int inputcheck = 1; //read digital inputs
int outputcheck = 1; //check outputs
int candebug = 0; //view can frames
int debugCur = 0;
int menuload = 0;
int balancecells;

ADC *adc = new ADC(); // adc object

void loadSettings()
{
  Logger::console("Resetting to factory defaults");
  settings.version = 0;
  settings.checksum = 0;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.0f;
  settings.ChargeVsetpoint = 4.1f;
  settings.DischVsetpoint = 3.2f;
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.DisTSetpoint = 40.0f;
  settings.IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5;//
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.logLevel = 2;
}

CAN_message_t msg;
CAN_message_t inMsg;

uint32_t lastUpdate;


void setup()
{
  delay(4000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's
  pinMode(ACUR1, INPUT);
  pinMode(ACUR2, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(OUT1, OUTPUT); // drive contactor
  pinMode(OUT2, OUTPUT); // precharge
  pinMode(OUT3, OUTPUT); // charge relay
  pinMode(OUT4, OUTPUT); // Negative contactor
  pinMode(OUT5, OUTPUT); // pwm driver output
  pinMode(OUT6, OUTPUT); // pwm driver output
  pinMode(OUT7, OUTPUT); // pwm driver output
  pinMode(OUT8, OUTPUT); // pwm driver output
  pinMode(led, OUTPUT);

  Can0.begin(500000);

  //if using enable pins on a transceiver they need to be set on


  adc->setAveraging(16); // set number of averages
  adc->setResolution(16); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->startContinuous(ACUR1, ADC_0);


  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALBMS.begin(612500); //Tesla serial bus
  //VE.begin(19200); //Victron VE direct bus
#if defined (__arm__) && defined (__SAM3X8E__)
  serialSpecialInit(USART0, 612500); //required for Due based boards as the stock core files don't support 612500 baud.
#endif

  SERIALCONSOLE.println("Started serial interface to BMS.");



  loadSettings();

  bms.renumberBoardIDs();

  Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;

  //bms.clearFaults();
  bms.findBoards();
  digitalWrite(led, HIGH);
  bms.setPstrings(Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
}

void loop()
{
  while (Can0.available())
  {
    canread();
  }
  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }

  //contcon();
  if (ESSmode == 1)
  {
    if (digitalRead(IN1) == LOW)//Key OFF
    {
      if (storagemode == 1)
      {
        settings.ChargeVsetpoint += storagedelta;
        settings.DischVsetpoint -= storagedelta;
        storagemode = 0;
      }
    }
    else
    {
      if (storagemode == 0)
      {
        settings.ChargeVsetpoint -= storagedelta;
        settings.DischVsetpoint += storagedelta;
        storagemode = 1;
      }
    }
    if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
    {
      bms.balanceCells();
      balancecells = 1;
    }
    else
    {
      balancecells = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      digitalWrite(OUT1, LOW);//turn off discharge
    }
    else
    {
      digitalWrite(OUT1, HIGH);//turn on discharge
    }

    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      digitalWrite(OUT3, LOW);//turn off charger
    }
    else
    {
      digitalWrite(OUT3, HIGH);//turn on charger
    }
  }
  else
  {
    switch (bmsstatus)
    {
      case (Boot):
        Discharge = 0;


        bmsstatus = Ready;
        break;

      case (Ready):
        Discharge = 0;
        if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
        {
          bms.balanceCells();
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }
        if (digitalRead(IN2) == HIGH && (settings.balanceVoltage + settings.balanceHyst) > bms.getHighCellVolt()) //detect AC present for charging and check not balancing
        {
          bmsstatus = Charge;
        }
        if (digitalRead(IN1) == HIGH) //detect Key ON
        {
          bmsstatus = Precharge;
          Pretimer = millis();
        }

        break;

      case (Precharge):
        Discharge = 0;
        Prechargecon();
        break;


      case (Drive):
        Discharge = 1;
        if (digitalRead(IN1) == LOW)//Key OFF
        {
          digitalWrite(OUT4, LOW);
          digitalWrite(OUT1, LOW);

          contctrl = 0; //turn off out 5 and 6
          bmsstatus = Ready;
        }

        break;

      case (Charge):
        Discharge = 0;
        digitalWrite(OUT3, HIGH);//enable charger
        if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
        {
          bms.balanceCells();
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }
        if (bms.getHighCellVolt() > settings.OverVSetpoint);
        {
          digitalWrite(OUT3, LOW);//turn off charger
          bmsstatus = Ready;
        }
        if (digitalRead(IN2) == LOW)//detect AC not present for charging
        {
          digitalWrite(OUT3, LOW);//turn off charger
          bmsstatus = Ready;
        }
        break;

      case (Error):
        Discharge = 0;

        if (digitalRead(IN2) == HIGH) //detect AC present for charging
        {
          bmsstatus = Charge;
        }
        if (bms.getLowCellVolt() >= settings.UnderVSetpoint)
        {
          bmsstatus = Ready;
        }

        break;
    }
  }
  if (cursens == Analogue)
  {
    getcurrent();
  }
  if (millis() - looptime > 500)
  {

    looptime = millis();
    bms.getAllVoltTemp();

    //UV  check

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      bmsstatus = Error;
    }


    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails();
    }
    if (inputcheck != 0)
    {
      inputdebug();
    }

    if (outputcheck != 0)
    {
      outputdebug();
    }
    updateSOC();
    currentlimit();
    VEcan();
  }
}

void alarmupdate()
{
  alarm[0] = 0;
  if (bms.getHighCellVolt() > settings.OverVSetpoint);
  {
    alarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    alarm[0] |= 0x10;
  }
  if (bms.getAvgTemperature() < settings.OverTSetpoint)
  {
    alarm[0] |= 0x40;
  }
  alarm[1] = 0;
  if (bms.getAvgTemperature() < settings.UnderTSetpoint)
  {
    alarm[1] = 0x01;
  }
}

void gaugeupdate()
{
  analogWrite(OUT8, map(SOC, 0, 100, gaugelow, gaugehigh));
  if (debug != 0)
  {
    SERIALCONSOLE.println("  ");
    SERIALCONSOLE.print("fuel pwm : ");
    SERIALCONSOLE.print(map(SOC, 0, 100, gaugelow, gaugehigh));
    SERIALCONSOLE.println("  ");
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  if (ESSmode == 1)
  {
    SERIALCONSOLE.print("ESS Mode ");

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      SERIALCONSOLE.print(": UnderVoltage ");
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      SERIALCONSOLE.print(": OverVoltage ");
    }
    if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
    {

      SERIALCONSOLE.print(": Happy ");

    }
  }
  else
  {
    SERIALCONSOLE.print(bmsstatus);
    switch (bmsstatus)
    {
      case (Boot):
        SERIALCONSOLE.print(" Boot ");
        break;

      case (Ready):
        SERIALCONSOLE.print(" Ready ");
        break;

      case (Precharge):
        SERIALCONSOLE.print(" Precharge ");
        break;

      case (Drive):
        SERIALCONSOLE.print(" Drive ");
        break;

      case (Charge):
        SERIALCONSOLE.print(" Charge ");
        break;

      case (Error):
        SERIALCONSOLE.print(" Error ");
        break;
    }
  }
  SERIALCONSOLE.print("  ");
  if (digitalRead(IN2) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(IN1) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
  if (balancecells == 1)
  {
    SERIALCONSOLE.print("|Balancing Active");
  }
}


void getcurrent()
{
  if (cursens == Analogue)
  {
    if (currentact < 19000 && currentact > -19000)
    {
      sensor = 1;
      adc->startContinuous(ACUR1, ADC_0);
    }
    else
    {
      sensor = 2;
      adc->startContinuous(ACUR2, ADC_0);
    }

    if (sensor == 1)
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("Low Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
        SERIALCONSOLE.print("  ");
      }
      RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset1) * 15.7;
      if (value < 100 || value > (adc->getMaxValue(ADC_0) - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
    else
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("High Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
        SERIALCONSOLE.print("  ");
      }
      RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset2) * highconv;
      if (value < 100 || value > (adc->getMaxValue(ADC_0) - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
  }
  if (invertcur == 1)
  {
    RawCur = RawCur * -1;
  }
  RunningAverageBuffer[NextRunningAverage++] = RawCur;
  if (NextRunningAverage >= RunningAverageCount)
  {
    NextRunningAverage = 0;
  }
  float RunningAverageCur = 0;
  for (int i = 0; i < RunningAverageCount; ++i)
  {
    RunningAverageCur += RunningAverageBuffer[i];
  }
  RunningAverageCur /= RunningAverageCount;

  currentact = RunningAverageCur;

  if (cursens == Analogue)
  {
    if (sensor == 1)
    {
      if (currentact > 500 || currentact < -500 )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
    if (sensor == 2)
    {
      if (currentact > 180000 || currentact < -18000 )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
  }
  else
  {
    if (currentact > 500 || currentact < -500 )
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }
  RawCur = 0;
}

void updateSOC()
{
  if (SOCset == 0)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), socvolt[0], socvolt[2], socvolt[1], socvolt[3]);
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("  ");
    ampsecond = (SOC * CAP * 10) / 0.27777777777778 ;
    SOCset = 1;
  }
  SOC = ((ampsecond * 0.27777777777778) / (CAP * 1000)) * 100;
  if (bms.getAvgCellVolt() > settings.OverVSetpoint)
  {
    ampsecond = (CAP * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    if (SOC >= 100)
    {
      SOC = 100;
    }
  }


  if (SOC < 0)
  {
    SOC = 0; //reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (debug != 0)
  {
    if (cursens == Analogue)
    {
      if (sensor == 1)
      {
        SERIALCONSOLE.print("Low Range ");
      }
      else
      {
        SERIALCONSOLE.print("High Range");
      }
    }
    else
    {
      SERIALCONSOLE.print("CANbus ");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.println ("mAh");

  }
}

void Prechargecon()
{
  if (digitalRead(IN1) == HIGH) //detect Key ON
  {
    digitalWrite(OUT4, HIGH);//Negative Contactor Close
    contctrl = 2;
    if (Pretimer + Pretime > millis() || currentact > Precurrent)
    {
      digitalWrite(OUT2, HIGH);//precharge
    }
    else //close main contactor
    {
      digitalWrite(OUT1, HIGH);//Positive Contactor Close
      contctrl = 3;
      bmsstatus = Drive;
      digitalWrite(OUT2, LOW);
    }
  }
  else
  {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT4, LOW);
    bmsstatus = Ready;
    contctrl = 0;
  }
}

void contcon()
{
  if (contctrl != contstat) //check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      //analogWrite(OUT5, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(OUT6, 0);
      contstat = contstat & 253;
    }

    if ((contctrl & 1) == 1)
    {
      if (conttimer == 0)
      {
        //analogWrite(OUT5, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        //analogWrite(OUT5, conthold);
        contstat = contstat | 1;
        conttimer = 0;
      }
    }

    if ((contctrl & 2) == 2)
    {
      if (conttimer == 0)
      {
        analogWrite(OUT6, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        analogWrite(OUT6, conthold);
        contstat = contstat | 2;
        conttimer = 0;
      }
    }
    /*
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */

  }
  if (contctrl == 0)
  {
    //analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
  }
}

void calcur()
{
  adc->startContinuous(ACUR1, ADC_0);
  sensor = 1;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset1 = offset1 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset1 = offset1 / 21;
  SERIALCONSOLE.print(offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->startContinuous(ACUR2, ADC_0);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset2 = offset2 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset2 = offset2 / 21;
  SERIALCONSOLE.print(offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}

void VEcan() //communication with Victron system over CAN
{
  msg.id  = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t((settings.ChargeVsetpoint * bms.seriescells() / Pstrings) * 10));
  msg.buf[1] = highByte(uint16_t((settings.ChargeVsetpoint * bms.seriescells() / Pstrings) * 10));
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent );
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((settings.DischVsetpoint * bms.seriescells() / Pstrings) * 10));
  msg.buf[7] = highByte(uint16_t((settings.DischVsetpoint * bms.seriescells() / Pstrings) * 10));
  Can0.write(msg);

  msg.id  = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(SOC * 10);
  msg.buf[5] = highByte(SOC * 10);
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  msg.id  = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  delay(2);
  msg.id  = 0x35A;
  msg.len = 8;
  msg.buf[0] = alarm[0];//High temp  Low Voltage | High Voltage
  msg.buf[1] = alarm[1]; // High Discharge Current | Low Temperature
  msg.buf[2] = alarm[2]; //Internal Failure | High Charge current
  msg.buf[3] = alarm[3];// Cell Imbalance
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can0.write(msg);

  msg.id  = 0x35E;
  msg.len = 8;
  msg.buf[0] = bmsname[0];
  msg.buf[1] = bmsname[1];
  msg.buf[2] = bmsname[2];
  msg.buf[3] = bmsname[3];
  msg.buf[4] = bmsname[4];
  msg.buf[5] = bmsname[5];
  msg.buf[6] = bmsname[6];
  msg.buf[7] = bmsname[7];
  Can0.write(msg);

  delay(2);
  msg.id  = 0x370;
  msg.len = 8;
  msg.buf[0] = bmsmanu[0];
  msg.buf[1] = bmsmanu[1];
  msg.buf[2] = bmsmanu[2];
  msg.buf[3] = bmsmanu[3];
  msg.buf[4] = bmsmanu[4];
  msg.buf[5] = bmsmanu[5];
  msg.buf[6] = bmsmanu[6];
  msg.buf[7] = bmsmanu[7];
  Can0.write(msg);
}

void BMVmessage()//communication with the Victron Color Control System over VEdirect
{
  lasttime = millis();
  x = 0;
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[0]);
  VE.write(9);
  VE.print(bms.getPackVoltage() * 1000, 0);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[2]);
  VE.write(9);
  VE.print(currentact);
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[4]);
  VE.write(9);
  VE.print(ampsecond * 0.27777777777778, 0); //consumed ah
  VE.write(13);
  VE.write(10);
  VE.write(myStrings[6]);
  VE.write(9);
  VE.print(SOC * 10); //SOC
  x = 8;
  while (x < 20)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x ++;
    VE.write(9);
    VE.write(myStrings[x]);
    x ++;
  }
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(0x50); //0x59
  delay(10);

  while (x < 44)
  {
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[x]);
    x ++;
    VE.write(9);
    VE.write(myStrings[x]);
    x ++;
  }
  /*
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[32]);
    VE.write(9);
    VE.print(bms.getLowVoltage()*1000,0);
    VE.write(13);
    VE.write(10);
    VE.write(myStrings[34]);
    VE.write(9);
    VE.print(bms.getHighVoltage()*1000,0);
    x=36;

    while(x < 43)
    {
     VE.write(13);
     VE.write(10);
     VE.write(myStrings[x]);
     x ++;
     VE.write(9);
     VE.write(myStrings[x]);
     x ++;
    }
  */
  VE.write(13);
  VE.write(10);
  VE.write("Checksum");
  VE.write(9);
  VE.write(231);
}

// Settings menu
void menu()
{


  incomingByte = Serial.read(); // read the incoming byte:

  if (menuload == 2)
  {
    switch (incomingByte)
    {


      case 99: //c for calibrate zero offset

        calcur();
        break;



      case 113: //c for calibrate zero offset

        menuload = 0;
        incomingByte = 115;
        break;

      case 115: //s for switch sensor
        if (cursens == Analogue)
        {
          cursens = Canbus;
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(" CANbus Current Sensor ");
          SERIALCONSOLE.println("  ");
        }
        else
        {
          cursens = Analogue;
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(" Analogue Current Sensor ");
          SERIALCONSOLE.println("  ");
        }
        break;


      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }
  if (menuload == 3)
  {
    switch (incomingByte)
    {
      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;

      case 102: //f factory settings
        loadSettings();
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        break;

      case 114: //r for reset
        ampsecond = 0;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Zeroed ");
        SERIALCONSOLE.println("  ");
        break;

      case 100: //d dispaly settings
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Over Voltage Setpoint - 1 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Under Voltage Setpoint - 2");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print(" Over Temperature Setpoint - 3");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print(" Under Temperature Setpoint - 4");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Setpoint - 5 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Hystersis - 6 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(CAP);
        SERIALCONSOLE.print("Ah Battery Capacity - 7 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(chargecurrentmax * 0.001);
        SERIALCONSOLE.print("A max Charge - 8 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(discurrentmax * 0.001);
        SERIALCONSOLE.print("A max Discharge - 9 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Charge Voltage Limit Setpoint - a ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Discharge Voltage Limit Setpoint - b");
        SERIALCONSOLE.println("  ");
        break;
      case 101: //e dispaly settings
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("Enter Variable Number and New value ");
        SERIALCONSOLE.println("  ");
        break;

      case 49: //1 Over Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.OverVSetpoint = Serial.parseInt();
          settings.OverVSetpoint = settings.OverVSetpoint / 1000;
          SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
          SERIALCONSOLE.print("mV Over Voltage Setpoint");
        }
        break;

      case 'a': //a Charge Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.ChargeVsetpoint = Serial.parseInt();
          settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
          SERIALCONSOLE.print(settings.ChargeVsetpoint  * 1000, 0);
          SERIALCONSOLE.print("mV Charge Voltage Limit Setpoint");
        }
        break;

      case 'b': //Discharge Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.DischVsetpoint = Serial.parseInt();
          settings.DischVsetpoint = settings.DischVsetpoint / 1000;
          SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
          SERIALCONSOLE.print("mV Discharge Voltage Limit Setpoint");
        }
        break;

      case 50: //2 Under Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderVSetpoint = Serial.parseInt();
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
          SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
          SERIALCONSOLE.print("mV Over Voltage Setpoint");
        }
        break;

      case 51: //3 Over Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.OverTSetpoint = Serial.parseInt();
          SERIALCONSOLE.print(settings.OverTSetpoint);
          SERIALCONSOLE.print(" Over Temperature Setpoint");
        }
        break;

      case 52: //4 Udner Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderTSetpoint = Serial.parseInt();
          SERIALCONSOLE.print(settings.UnderTSetpoint);
          SERIALCONSOLE.print(" Under Temperature Setpoint");
        }
        break;

      case 53: //5 Balance Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Setpoint");
        }
        break;

      case 54: //6 Balance Voltage Hystersis
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Hystersis");
        }
        break;

      case 55://7 Battery Capacity inAh
        if (Serial.available() > 0)
        {
          CAP = Serial.parseInt();
          SERIALCONSOLE.print(CAP);
          SERIALCONSOLE.print("Ah Battery Capacity");
        }
        break;
      case 56://8 chargecurrent A
        if (Serial.available() > 0)
        {
          chargecurrentmax = Serial.parseInt() * 10;
          SERIALCONSOLE.print(chargecurrentmax * 0.1);
          SERIALCONSOLE.print("A max Charge");
        }
        break;
      case 57://9 discurrent in A
        if (Serial.available() > 0)
        {
          discurrentmax = Serial.parseInt() * 10;
          SERIALCONSOLE.print(discurrentmax * 0.1);
          SERIALCONSOLE.print("A max Discharge");
        }
        break;

    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 'r'://restart
        CPU_REBOOT ;
        break;

      case 113: //q to go back to main menu

        menuload = 0;
        debug = 1;
        break;

      case 99: //c for calibrate zero offset
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Current Sensor Calibration Menu");
        SERIALCONSOLE.println("c - To calibrate sensor offset");
        SERIALCONSOLE.println("s - To switch between Current Sensors");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 2;
        break;

      case 98: //c for calibrate zero offset
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Battery Settings Menu");
        SERIALCONSOLE.println("r - Reset AH counter");
        SERIALCONSOLE.println("d - Display settings");
        SERIALCONSOLE.println("e - Edit settings");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 3;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (incomingByte == 115 & menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("R - Restart BMS");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

void canread()
{
  Can0.read(inMsg);
  // Read data: len = data length, buf = data byte(s)
  switch (inMsg.id)
  {
    case 0x3c2:
      CAB300();
      break;

    default:
      break;
  }
  if (candebug == 1)
  {
    Serial.print(millis());
    if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
    else
      sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

    Serial.print(msgString);

    if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i < inMsg.len; i++) {
        sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
        Serial.print(msgString);
      }
    }

    Serial.println();
  }
}

void CAB300()
{
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000)
  {
    CANmilliamps -= 0x80000000;
  }
  else
  {
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  }
  if (cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void currentlimit()
{
  if (bmsstatus == Error)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  else
  {
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    else
    {
      if (bms.getAvgTemperature() < settings.ChargeTSetpoint)
      {
        discurrent = discurrentmax;
        chargecurrent = map(bms.getAvgTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, 0, chargecurrentmax);
      }
      else
      {
        if (bms.getAvgTemperature() < settings.DisTSetpoint)
        {
          discurrent = discurrentmax;
          chargecurrent = chargecurrentmax;
        }
        else
        {
          if (bms.getAvgTemperature() < settings.OverTSetpoint)
          {
            discurrent = map(bms.getAvgTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, discurrentmax, 0);
            chargecurrent = chargecurrentmax;
          }
          else
          {
            discurrent = 0;
            chargecurrent = 0;
          }
        }
      }
    }
  }
}

void inputdebug()
{
  Serial.println();
  Serial.print("Input: ");
  if (digitalRead(IN1))
  {
    Serial.print("1 ON  ");
  }
  else
  {
    Serial.print("1 OFF ");
  }
  if (digitalRead(IN2))
  {
    Serial.print("2 ON  ");
  }
  else
  {
    Serial.print("2 OFF ");
  }
  if (digitalRead(IN3))
  {
    Serial.print("3 ON  ");
  }
  else
  {
    Serial.print("3 OFF ");
  }
  if (digitalRead(IN4))
  {
    Serial.print("4 ON  ");
  }
  else
  {
    Serial.print("4 OFF ");
  }
  Serial.println();
}

void outputdebug()
{
  if (outputstate < 5)
  {
    digitalWrite(OUT1, HIGH);
    digitalWrite(OUT2, HIGH);
    digitalWrite(OUT3, HIGH);
    digitalWrite(OUT4, HIGH);
    digitalWrite(OUT5, HIGH);
    digitalWrite(OUT6, HIGH);
    digitalWrite(OUT7, HIGH);
    digitalWrite(OUT8, HIGH);
    outputstate ++;
  }
  else
  {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT3, LOW);
    digitalWrite(OUT4, LOW);
    digitalWrite(OUT5, LOW);
    digitalWrite(OUT6, LOW);
    digitalWrite(OUT7, LOW);
    digitalWrite(OUT8, LOW);
    outputstate ++;
  }
  if (outputstate > 10)
  {
    outputstate = 0;
  }
}

