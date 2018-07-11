#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      modules[y].clearmodule();
    }
  }
}

int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::balanceCells()
{
  uint8_t payload[4];
  uint8_t buff[30];
  uint8_t balance = 0;//bit 0 - 5 are to activate cell balancing 1-6

  for (int address = 1; address <= MAX_MODULE_ADDR; address++)
  {
    if (modules[address].isExisting())
    {
      balance = 0;
      for (int i = 0; i < 6; i++)
      {
        if (getLowCellVolt() < modules[address].getCellVoltage(i))
        {
          balance = balance | (1 << i);
        }
      }

      if (balance != 0) //only send balance command when needed
      {
        payload[0] = address << 1;
        payload[1] = REG_BAL_TIME;
        payload[2] = 0x05; //5 second balance limit, if not triggered to balance it will stop after 5 seconds
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);

        payload[0] = address << 1;
        payload[1] = REG_BAL_CTRL;
        payload[2] = balance; //write balance state to register
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);

        if (Logger::isDebug()) //read registers back out to check if everthing is good
        {
          delay(50);
          payload[0] = address << 1;
          payload[1] = REG_BAL_TIME;
          payload[2] = 1; //
          BMSUtil::sendData(payload, 3, false);
          delay(2);
          BMSUtil::getReply(buff, 30);

          payload[0] = address << 1;
          payload[1] = REG_BAL_CTRL;
          payload[2] = 1; //
          BMSUtil::sendData(payload, 3, false);
          delay(2);
          BMSUtil::getReply(buff, 30);
        }
      }
    }
  }
}

/*
   Try to set up any unitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
   still at least one unitialized board. Go ahead and give it the first ID not registered as already taken.
   If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
   Don't run this routine until after the boards have already been enumerated.\
   Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
   To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
   address and see if any boards respond back saying that they set the address.
*/
void BMSModuleManager::setupBoards()
{
  uint8_t payload[3];
  uint8_t buff[10];
  int retLen;

  payload[0] = 0;
  payload[1] = 0;
  payload[2] = 1;

  while (1 == 1)
  {
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;
    retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
    if (retLen == 4)
    {
      if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
      {
        Logger::debug("00 found");
        //look for a free address to use
        for (int y = 1; y < 63; y++)
        {
          if (!modules[y].isExisting())
          {
            payload[0] = 0;
            payload[1] = REG_ADDR_CTRL;
            payload[2] = y | 0x80;
            BMSUtil::sendData(payload, 3, true);
            delay(3);
            if (BMSUtil::getReply(buff, 10) > 2)
            {
              if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80))
              {
                modules[y].setExists(true);
                numFoundModules++;
                Logger::debug("Address assigned");
              }
            }
            break; //quit the for loop
          }
        }
      }
      else break; //nobody responded properly to the zero address so our work here is done.
    }
    else break;
  }
}

/*
   Iterate through all 62 possible board addresses (1-62) to see if they respond
*/
void BMSModuleManager::findBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];

  numFoundModules = 0;
  payload[0] = 0;
  payload[1] = 0; //read registers starting at 0
  payload[2] = 1; //read one byte
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    modules[x].setExists(false);
    payload[0] = x << 1;
    BMSUtil::sendData(payload, 3, false);
    delay(20);
    if (BMSUtil::getReply(buff, 8) > 4)
    {
      if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0) {
        modules[x].setExists(true);
        numFoundModules++;
        Logger::debug("Found module with address: %X", x);
      }
    }
    delay(5);
  }
}


/*
   Force all modules to reset back to address 0 then set them all up in order so that the first module
   in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
  uint8_t payload[3];
  uint8_t buff[8];
  int attempts = 1;

  for (int y = 1; y < 63; y++)
  {
    modules[y].setExists(false);
    numFoundModules = 0;
  }

  while (attempts < 3)
  {
    payload[0] = 0x3F << 1; //broadcast the reset command
    payload[1] = 0x3C;//reset
    payload[2] = 0xA5;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(100);
    BMSUtil::getReply(buff, 8);
    if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
    attempts++;
  }

  setupBoards();
}

/*
  After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Alert Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_FAULT_STATUS;//Fault Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  isFaulted = false;
}

/*
  Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state.
  Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x04;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

/*
  Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x00;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Fault Status
  payload[2] = 0x04;//data to cause a reset
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::getAllVoltTemp()
{
  packVolt = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      Logger::debug("");
      Logger::debug("Module %i exists. Reading voltage and temperature values", x);
      modules[x].readModuleValues();
      Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
      Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
      Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

  if (digitalRead(11) == LOW) {
    if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) Logger::info("All modules have exited a faulted state");
    isFaulted = false;
  }
}

float BMSModuleManager::getLowCellVolt()
{
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }
  return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore)
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAvgTemp() > -70)
      {
        avg += modules[x].getAvgTemp();
      }
      else
      {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getAvgCellVolt()
{
  float avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting()) avg += modules[x].getAverageV();
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      if (faults > 0)
      {
        Logger::console("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          SERIALCONSOLE.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (COV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 2)
        {
          SERIALCONSOLE.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (CUV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 4)
        {
          Logger::console("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Logger::console("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Logger::console("    Test fault active");
        }
        if (faults & 0x20)
        {
          Logger::console("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Logger::console("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Logger::console("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Logger::console("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Logger::console("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Logger::console("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Logger::console("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Logger::console("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Logger::console("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Logger::console("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) SERIALCONSOLE.println();
    }
  }
}

void BMSModuleManager::printPackDetails()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  int cellNum = 0;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV   Avg Temp: %fC ", numFoundModules, seriescells(),
                  Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      SERIALCONSOLE.print("Module #");
      SERIALCONSOLE.print(y);
      if (y < 10) SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(modules[y].getModuleVoltage());
      SERIALCONSOLE.print("V");
      for (int i = 0; i < 6; i++)
      {
        if (cellNum < 10) SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print("  Cell");
        SERIALCONSOLE.print(cellNum++);
        SERIALCONSOLE.print(": ");
        SERIALCONSOLE.print(modules[y].getCellVoltage(i));
        SERIALCONSOLE.print("V");
      }
      SERIALCONSOLE.print("  Neg Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print("C  Pos Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.println("C");

    }
  }
}
