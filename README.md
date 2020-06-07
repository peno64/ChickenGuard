# Kippenluik - Chicken guard

This project is derived from code I found on the internet (Techniek & Dier).


An Arduino uno is used as controller to make a hatch open and close when the day starts and ends.
This to secure the chickens in the henhouse from foxes and marten.

The code was modified quite a bit from the original code because my hatch is different from the original one and I added extra functionality.
It is a trap door (like from a medieval castel).
A magnetic switch detects when the door is closed in opposite from the original design where it detects when the door is open.
So to close the door it can detect when it is closed but to open it this must be done with a timer.
I also use an Elastic instead of a washing thread to pull the door such that there is some elastic on it when it overruns.
Also at night it also checks every second if the door is still closed. If not then it closes the door again.
This in case the agressor tries to open the door...
When the door is closed, it closes again a bit for some milliseconds to have it good closed. This can be becaise of the elastic.
However it can also happen that the motor cannot hold the elastic and it returns a bit and as such it is tried 10 times.
Also here is a 2 colored led at the outside which will show the status.
If the door is open then the red led is on. If the door is closed then it is the green led that is on. green for safe, they are locked, red for possible unsafe.
a couple minutes before opening and closing the corresponding leds start to blink to show that the door is about to open/close.
Also there is the possibility of a clock module (DS3231). It is optional. If it is there then it can be used to set to minimum time the door may open.
Otherwise in the summer it already opens at 5h which is very early and predators could still give a visit that early. I use it to specify that the door may not
open before 7h30 in the morning. Even if that clock module is not there it is possible to do this also with the timer from the arduino but this timer is not
very precise such that you will need to reset the time regularly (every week or so).
