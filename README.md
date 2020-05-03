# Kippenluik - Chicken guard

This project is derived from code I found on the internet (Techniek & Dier)

An Arduino uno is used as controller to make a hatch open and close when the day starts and ends.
This to secure the chickens in the henhouse from foxes and marten.

The code was modified quite a bit from the original code because my hatch is different from the original one. 
It is a trap door (like from a medieval castel). 
A magnetic switch detects when the door is closed in opposite from the original design where it detects when the door is open.
I also use an Elastic instead of a washing thread to pull the door such that there is some elastic on it when it overruns.
Also at night it also checks every second if the door is still closed. If not then it closes the door again. 
This in case the agressor tries to open the door...
