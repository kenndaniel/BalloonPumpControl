// #define NEW_BALLOON 1
// #define RESTART 0


// bool correct_command = false;
// char serial_command_buffer_[32];
// SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

// //This is the default handler, and gets called when no other command matches. 
// // Note: It does not get called for one_key commands that do not match
// void cmd_unrecognized(SerialCommands* sender, const char* cmd)
// {
//   sender->GetSerial()->print("Unrecognized command [");
//   sender->GetSerial()->print(cmd);
//   sender->GetSerial()->println("]");
// }

// //called for starting a new balloon 'n' command
// void cmd_zero(SerialCommands* sender)
// {
//   sender->GetSerial()->println("Starting from zero pressure");
//   presSetup(false);
//   correct_command = true;
// }

// //called for continuing to pressurize the current balloon 'r' command
// void cmd_restart(SerialCommands* sender)
// {
//   sender->GetSerial()->println("Continuing to pressurize from last time");
//   presSetup(false);
//   correct_command = true;
// }

// //Note: Commands are case sensitive

// // Add one_key commands to call the same on and off function but by simply
// // pressing '1' or '0' without needing a terminating key stroke.
// // One_key commands are ONLY tested when they are the first key pressed on
// // startup, or the first key after a previous terminating keystroke.
// SerialCommand cmd_restart_ok_("r", cmd_restart, true);
// SerialCommand cmd_zero_ok_("n", cmd_zero, true);


// void setupCommands() 
// {

//   serial_commands_.SetDefaultHandler(cmd_unrecognized);
//   serial_commands_.AddCommand(&cmd_zero_ok_);
//   serial_commands_.AddCommand(&cmd_restart_ok_);


// }


