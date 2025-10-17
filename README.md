# BD_Pressure

### Features:

#### 1. Automated Pressure Advanced Calibration

#### 2. Nozzle Probe

<img src="https://static.wixstatic.com/media/0d0edf_1ebb592e9ab04beeacb07abdf56b3e41~mv2.jpg/v1/fill/w_1658,h_1604,al_c,q_85,usm_0.66_1.00_0.01/0d0edf_1ebb592e9ab04beeacb07abdf56b3e41~mv2.jpg" width='600'>
<img src="https://static.wixstatic.com/media/0d0edf_08390d0d8b2947fb9dc51fb136b01c01~mv2.jpg/v1/fill/w_1338,h_802,al_c,q_85,usm_0.66_1.00_0.01/0d0edf_08390d0d8b2947fb9dc51fb136b01c01~mv2.jpg" width='400'>

#### How it works?

1. PA Mode:
 
    Automated Pressure Advanced Calibration. Without printing calibration lines, it just simulate extrusion pressure behavior during acceleration and deceleration while only the extruder is working. This work process is similar to the Bambu Lab A1 printer, instead, we use  strain gauge, not eddy sensor.

2. Nozzle Probe Mode:

     Use the strain gauge to sense the nozzle pressure while probing.It works as a normal switch endstop sensor, so we can just power it and connect the Z- pin on the mainboard. 

### klipper 

#### 1. Install software module
```
cd  ~
git clone https://github.com/markniu/bd_pressure.git
chmod 777 ~/bd_pressure/klipper/install.sh
~/bd_pressure/klipper/install.sh
```

#### 2. Configure Klipper

##### Upload the bd_pressure.cfg into the printer config folder and add [include bd_pressure.cfg] into the printer.cfg , 
```
[include bd_pressure.cfg]
```
##### Edit the bd_pressure.cfg : 
  Change the pins to your actual use in the section [probe] 
  , Choose the port(i2c or usb) used in the section [bdpressure bd_pa]

#### 3. OrcaSlicer:

1. Disable the Pressure advance in the Material settings.

2. Add the following G-code lines into the beginning of the Start_Gcode in the slicer, then it will do pressure advance calibration with your setting and automatically set the right PA value. 
```
G28                    ; Home all the axis
G1 Z30                 ; Move to the poop position. You can modify it depending on your printer. 
G1 X240 Y240           ; Move to the poop position. You can modify it depending on your printer.
PA_CALIBRATE NOZZLE_TEMP=[first_layer_temperature] MAX_VOLUMETRIC=[filament_max_volumetric_speed] ACC_WALL=[outer_wall_acceleration]  TRAVEL_SPEED=[travel_speed]  ACC_TO_DECEL_FACTOR=[accel_to_decel_factor]
```

####  Compatible:   
It is compatible with most hotends, but need to design a mount with CAD for different hotend.

### others
Store: https://www.pandapi3d.com/product-page/bdpressure

video: [test video](https://youtu.be/zLuWcR-ahno) ; [mount with bambu hotend](https://youtu.be/fwx00GEvlms)

Discord:  [3D Printscape](https://discord.gg/xn5rYHyE)
