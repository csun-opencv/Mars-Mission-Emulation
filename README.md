# Project Title

Mars Mission Emulation Project 

## Description

An object detection drone wirelessly commanding a wheeled robot to perform tasks, mimicking the NASA's Mars Exploration Mission.


## Dependencies


### Hardware

* [Hawk's Work F450](https://www.amazon.com/HAWKS-WORK-Controller-Unassembled-Quadcopter/dp/B09SZ7LNXB/ref=asc_df_B09SZ7LNXB/?tag=hyprod-20&linkCode=df0&hvadid=678719857173&hvpos=&hvnetw=g&hvrand=8287373968535265209&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9031153&hvtargid=pla-1931934789257&psc=1&mcid=8d5321886ba23dd8bc74ffec259c8b0c) is used as main drone platform projects.
    * **NOTE: The Pixhawk provided in this kit is NOT an official Pixhawk from Holybro and therefore the default firmware should NEVER be changed.**
* [Raspberry Pi 4 8GB kit](https://www.amazon.com/CanaKit-Raspberry-8GB-Starter-Kit/dp/B08956GVXN/ref=sr_1_4?keywords=raspberry+pi+4+8GB&qid=1701127316&sr=8-4&ufe=app_do%3Aamzn1.fos.304cacc1-b508-45fb-a37f-a2c47c48c32f) is used for the Raspberry Pi build.
* [Raspberry Pi 4000mAh Battery](https://www.amazon.com/VGE-Battery-Raspberry-4000mAh-Adhesive/dp/B09BNRKQD8/ref=sr_1_4?crid=P2VNN1525M8C&keywords=raspberry+pi+external+battery&qid=1701130051&s=electronics&sprefix=raspberry+pi+extermal+battery%2Celectronics%2C136&sr=1-4)
* [Google Coral](https://coral.ai/products/accelerator/)
* [Pi Camera V2](https://www.raspberrypi.com/products/camera-module-v2/)
* [RSLK Max Robot Chassis](https://www.pololu.com/product/3670)
* [MSP432P401R LaunchPad](https://www.amazon.com/Development-Boards-Kits-MSP432P401R-LaunchPad/dp/B01LWR1MSO) mounted on the chassis.
* [Sharp GP2Y0A21YK0F Analog Distance Sensor](https://www.pololu.com/product/136)
* [HC-05 Bluetooth Module](https://www.amazon.com/HiLetgo-Wireless-Bluetooth-Transceiver-Arduino/dp/B071YJG8DR)
* [Testing Cones](https://www.amazon.com/dp/B000TVK3U2?psc=1&ref=ppx_yo2ov_dt_b_product_details)

### Software

* On the Raspberry Pi side, the project was developed on Ubuntu 20.04 with Python 3.8.10
    * [System Packages](https://github.com/csun-opencv/Mars-Mission-Emulation/blob/main/dependencies/ubuntu_20_04_apt_packages.txt) and [pip packages](https://github.com/csun-opencv/Mars-Mission-Emulation/blob/main/dependencies/requirements_3_8_10.txt) are included in the [dependencies/](https://github.com/csun-opencv/Mars-Mission-Emulation/tree/main/dependencies) directory.
* On the robot side, [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO) was used to develop.
    * Inside Code Composer Studio, the project in the [ground_platform_ws/](https://github.com/csun-opencv/Mars-Mission-Emulation/tree/main/ground_platform_ws) directory can be imported as a CCS project.


## Documentation

* A full [report](https://github.com/csun-opencv/Mars-Mission-Emulation/blob/main/docs/Fall23_Mars_Mission_Emulation_Framework_ECE_493_Senior_Paper.pdf) can be found in the [docs/](https://github.com/csun-opencv/Mars-Mission-Emulation/tree/main/docs) directory where full project details are explained. 
* A PowerPoint [presentation](https://github.com/csun-opencv/Mars-Mission-Emulation/blob/main/media/Mars_Mission_Emulation_Presentation_Demo.pptx) covers the main points of the project.
* A simplified high-level [schematics](https://github.com/csun-opencv/Mars-Mission-Emulation/tree/main/schematics) of the major hardware used shows the main connections on each platform.
    
## Authors

* Abdullah Hendy
    * [@LinkedIn](https://www.linkedin.com/in/abdullah-hendy/)
    * [@GitHub](https://github.com/AbdullahHendy)
* Michael Granberry
    * [@LinkedIn](https://www.linkedin.com/in/michaelgranberryii/)
    * [@GitHub](https://github.com/michaelgranberryii)

## Version History

* 0
    * Initial version
