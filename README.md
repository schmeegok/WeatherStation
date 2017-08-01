# WeatherStation
Updates to my original station to post data to weather underground and sparkfun

Updated the code to have the Raspberry pi query the MicroView to get the measurements only when it needs them.
The previous version of the station had the MicroView constantly dumping serial measurements and the Raspberry Pi had the job of
pulling off the data once a second and displaying it.  Then every minute it would update the websites.

I wanted to change it to allow the RPi to query the values once a minute and post them at that point.  Then the Microview can continue taking measurements as fast as it can and display
them on the OLED.  Now I want to use the button to reset statistics instead of changing modes.  I will have the RPi change modes every 10 seconds or so.

