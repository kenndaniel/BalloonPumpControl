void processTerminalCommands()
{
  Serial.println(" Process Terminal Commands ");
  if( commandBuffer.startsWith(F("setpoint")) )  // change the setpoint to xx psi
  {
    const char * ptr = commandBuffer.getNextToken(" "); // read first argument
    if(ptr == NULL) 
    {
    Serial.println("Setpoint not changed! - format is setpoint .xx ");
    }
    else
    {
    String stpt(ptr);
    setpoint = stpt.toFloat();
    Serial.print(F("\nSetpoint updated \n"));
    Serial.println(setpoint);
    }


  }
  else if( commandBuffer.startsWith(F("ts")) ) // time show [N times]
  {
    // show time command
    const char * ptr = commandBuffer.getNextToken(" "); // read first argument
    if(ptr == NULL) {
      // simple show time
      //printTime(false);
    }
    else
    {
      // show time N times with 1sec interval
      //numShowTime = atoi(ptr);
    }
    
  }

  else if( commandBuffer.startsWith(F("activate")) )  // activate auto start mode
  {
    Serial.print(F("Activated"));
    //WorkModeActive = true;

  }

  
}

