# Arduino Proximity Gate-Opener
Arduino based garage opener for my home garage door.
The project consists of 2 parts: 
- A transimtter (Beacon) - a device placed in the car and transmits a signal in fixed intervals.
- A receiver - The reciever controls the garage door and opens/closes it depending on the time that have passed since last received transmission.

The project also uses:
- RC Switch - for rf transmission
- EEPROM for storing and restoring state and saved codes

This project uses my library [`ArduinoComponents`](https://github.com/gilmaimon/ArduinoComponents)
