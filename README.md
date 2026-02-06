# BD_Pressure
As we know, different filaments, printing speeds, temperatures, etc. have different PA values that affect print quality significantly. Until now, very few printers on the market have automated PA calibration; many still require manual calibration, which is time-consuming and not everyone can do.

<img src="https://static.wixstatic.com/media/0d0edf_f6d3465971184537b12e430bce7a0aea~mv2.jpg/v1/fill/w_841,h_758,al_c,q_85,usm_0.66_1.00_0.01/0d0edf_f6d3465971184537b12e430bce7a0aea~mv2.jpg" width='800'>
<img src="https://static.wixstatic.com/media/0d0edf_eee5984961de44a7bad4c91010afd43b~mv2.jpg/v1/crop/x_10,y_43,w_1744,h_886/fill/w_846,h_430,al_c,q_85,usm_0.66_1.00_0.01,enc_avif,quality_auto/vsV.jpg" width =800>

### Features:

1. Automated Pressure Advanced Calibration
2. Nozzle Probe
3. All in one: No external PCB board, USB/I2C,
4. Compatible with E3D/Voron printers


#### How it works?
1. PA Mode:
 Without printing calibration lines, it just simulate extrusion pressure behavior during acceleration and deceleration while only the extruder is working. This work process is similar to the Bambu Lab A1 printer, instead, I use  strain gauge, not eddy sensor.
2. Nozzle Probe Mode:
 Use the strain gauge to sense the nozzle pressure while probing.It works as a normal switch endstop sensor, so we can just power it and connect the Z- pin on the mainboard. 
<img src="https://static.wixstatic.com/media/0d0edf_1ebb592e9ab04beeacb07abdf56b3e41~mv2.jpg/v1/fill/w_1658,h_1604,al_c,q_85,usm_0.66_1.00_0.01/0d0edf_1ebb592e9ab04beeacb07abdf56b3e41~mv2.jpg" width='800'>

[Wiki Installation](https://pandapi3d.cn/en/bdpressure/home)


Store: [Pandapi3d](https://www.pandapi3d.com/product-page/bdpressuree)  、 [淘宝](https://item.taobao.com/item.htm?id=1008161962242&spm=a213gs.v2success.0.0.1b8548315JmReI)

[Youtube](https://youtu.be/6vDijDzVqsA) 

Discord: https://discord.gg/z6ahddnGVU 
