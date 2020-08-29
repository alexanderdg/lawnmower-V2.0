#include "Lawnmower.h"
#include "StateMachineImp.h"

Lawnmower::Lawnmower() {


}

void Lawnmower::run(void) {
  SM.RunStatemachine();
}

void Lawnmower::init(void) {
  SM.initStatemachine();
  Serial3.begin(115200);
}

void Lawnmower::checkBluetooth(void) {
  if (Serial3.available() > 0) {
    char temp = Serial3.read();
    Serial3.print(temp);
    if (temp == 13)
    {
      Serial3.println();
      char tempchar[30];
      char * pch;
      receivedText.toCharArray(tempchar, 30);
      pch = strchr(tempchar, ' ');
      String commandText = "";
      String dataText = "";
      if (pch != NULL) {
        Serial3.println(pch - tempchar + 1);
        commandText = receivedText.substring(0, pch - tempchar);
        if (receivedText.charAt(pch - tempchar + 1) != ' ') dataText = receivedText.substring(pch - tempchar + 1);
      }
      else {
        commandText = receivedText;
      }

      if (commandText == "s")
      {
        SM.updateState(RUN);
      }
      else if (commandText == "h")
      {
        SM.updateState(INIT);
      }
      else if (commandText == "t")
      {
        SM.updateState(RETURN_HOME);
      }
      else if (commandText == "d")
      {
        SM.printDiagnostics();
      }
      else if (commandText == "PID")
      {
        SM.printPIDValues();

      }
      else if (commandText == "PID_P")
      {
        Serial3.print("Change PID P setting to ");
        Serial3.println(dataText);
        SM.changePValue(dataText.toFloat());
      }
      else if (commandText == "PID_I")
      {
        Serial3.print("Change PID I setting to ");
        Serial3.println(dataText);
        SM.changeIValue(dataText.toFloat());
      }
      else if (commandText == "PID_D")
      {
        Serial3.print("Change PID D setting to ");
        Serial3.println(dataText);
        SM.changeDValue(dataText.toFloat());
      }
      else if (commandText == "PID_T")
      {
        Serial3.print("Change PID Setpoint to ");
        Serial3.println(dataText);
        SM.changePIDSetpoint(dataText.toFloat());
      }
      else if (commandText == "HELP" or commandText == "Help" or commandText == "help")
      {
        printHelpMenu();
      }


      receivedText = "";
    }
    else
    {
      receivedText += temp;
    }

  }
}


void Lawnmower::printHelpMenu(void)
{
  Serial3.println("---------------------------------------------------------");
  Serial3.println("---                    HelpMenu                       ---");
  Serial3.println("---------------------------------------------------------");
  Serial3.println("Command 'HELP' : Shows this helpmenu");
  Serial3.println("Command 'h' : Stops direct the mower");
  Serial3.println("Command 'd' : Ask for diagnostics of the mower");
  Serial3.println("Command 's' : Manual start the mower at own risk!");
  Serial3.println("Command 't' : Find the perimeter at own risk!");
  Serial3.println("Command 'PID' : Shows the PID setting of the return to home algoritme");
  Serial3.println("Command 'PID_P' : CHange PID P setting of the return to home algoritme");
  Serial3.println("Command 'PID_I' : CHange PID I setting of the return to home algoritme");
  Serial3.println("Command 'PID_D' : CHange PID D setting of the return to home algoritme");
  Serial3.println("---------------------------------------------------------");
}
