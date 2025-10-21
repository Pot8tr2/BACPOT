Create a scheme that takes in values from uart, and make it move specifically with each command. 
Check how many ticks are in a rotation. Check that it is the same on each wheel. It should roughly be around 2800. 
Change over the pins for the interrupts to the ones isted in the pcb document. This should be available from the schematic.This means you will have to change the order of the interrupts to match lines A and B work for each motor in specific. Ie the specific pins. 
For the Turns have it start turning quickly and slow down as it gets closer to the specified degree angle. You can start out with this for 90 degrees. Implement Pid for the turns so it slows down as it gets closer to 90 degrees. Ie turns quickly and then more slowly. 

