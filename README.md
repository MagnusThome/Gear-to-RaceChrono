# Gear-to-RaceChrono
Get calculated gear plus rpm and speed at 20Hz from CAN/OBD2 to RaceChrono
  
**Work in progress** and to be honest a bit of a mess put together from different sources. Any board based on ESP32 or ESP32-S3 should work with the addition of a CAN transceiver to hook up to your car's OBD2 or CAN bus. It all works fine but for very slow shifts, where the clutch is depressed for a long time, there is some filtering in the code that needs fine tuning to prevent showing random erroneous gears when the clutch is pushed down and the car has no gear engaged. 
  
Thanks to:  
https://github.com/NicoEFI/Racechrono-ESP32-S3  
  
##  Setting up in Racechrono  

In Racechrono add the DIY BLE under "Add other device".    
Then add the following PIDs:  
0x20 (16) = Gear  
0x21 (17) = Speed kmh  
0x22 (18) = Engine RPM  
  
You can also add this to get a log of your car's rpm/kmh ratios to use to set the gear ratio thresholds in the main code.  
0x23 (19) = ratio rpm/kmh  
You might get away with the default ratio values, at least for lower gears. But you probably need to edit the threshold ratios[] array at the top of main.ino to work with your car's ratios. 
