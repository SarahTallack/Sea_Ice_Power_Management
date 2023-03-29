# Sea Ice Power Management

## Overview: 

The below extract is from the vacwork advertisement: 

“This work feeds into the physical characterisation of sea ice using LiDAR and cameras to improve parameterisation of Earth System Models. A preliminary prototype rig was constructed for the SCALE Winter 2022 cruise to collect data from a Livox AVIA LiDAR (https://www.livoxtech.com/avia) and an Intel Realsense D455 Depth camera (https://www.intelrealsense.com/depth-camera-d455). During the SCALE cruise we were able to collect a comprehensive LiDAR/Realsense scan dataset of individual pancake ice floes which were collected and placed on deck. The goal of these vacation work positions is to improve the reliability and usability of the imaging rig to make it more robust for future experiments and cruises.” 

This project focuses on the design and improvement of the power management for the rig mentioned in the above. This report aims to outline the process taken to understand the power requirements of the system, as well as to research possible methods for supplying power in cold weather (-20°C). 

## Design Process and Decisions 

Before exploring methods for supplying power, it is important to understand the power requirements of the system. Thus, the first step is to research individual components and determine what voltages they are able to operate at and draw up a power budget for the system. The power budget for the system is shown in the following section. 

Once a power budget had been compiled, a meeting was had to discuss the best methods for supplying power to the system. It was decided that a battery supply such a EcoFlow or Jackery would be the most robust and allow other groups to use the supply if needed. Since these batteries are Lithium-Ion, which operate best at room-temperature, a temperature controller enclosure needs to be designed for the battery. Additionally, the team wanted the system to be able to be powered from mains or from the battery, and to be able to easily switch between these two options. 

Thus, two separate circuits need to be designed. The first should be able to distribute a single supply to at least two devices, as well as to switch between either mains or a battery as the power supply. The second design needed to control the temperature of the box, using a microcontroller, sensors and heating pads. 

## Power Budget

The power budget can be found in an excel spreadsheet called PowerBudget.

## Power Distribution

### Basic Operation

Two devices needed to be power by a single source, namely an Intel NUC (mini-PC) and a Livox AVIA. These devices needed to either be powered by a main supply, via an AC/DC supply, or by a DC output from a battery supply. 

The user should be able to switch between the two supplies and turn everything off. A panel mounted switch is to be connected via the switch header pins (J6). An external LED indicator is connected in a similar fashion, using the J7 pins.  

A MOSFET configuration is used for switching when power is supplied, and the switch is ON. When Q1 is turned on, it allows Q2 to be turned on, which then provides power to the external jacks. Fuses are added to the output of each supply. 

Component Selection 

The input jacks had to be selected to ensure they could withstand up to 8A. 8A fuses are placed to ensure the circuit is not overloaded. Schottky diodes are chosen that can withstand the current to be supplied. 

N-channel MOSFETs are chosen for low current switching, and the P-channel MOSFET is selected as it can withstand high currents. The DC jacks at the output are selected to withstand up to 5A of current, which is the maximum current draw of the devices to be powered. 

### KiCAD project files

The project files for this board can be found in "\PCB Design\Power Distribution".

## Temperature Controller

### Basic Operation

The PCB is designed to fit onto an STM32 Nucleo-64 microcontroller. It is designed to read a temperature controller and implement a bang-bang controller to maintain the ambient temperature in an enclosure between a lower and upper temperature value, by switching external heating pads on or off.  

An RTD sensor is used in a Wheatstone bridge configuration, and the two outputs of the bridge are sent into an instrumentation amplifier (U1). The output of the instrumentation amplifier is sent to an ADC pin on the Nucleo. The Nucleo then converts this ADC value into a temperature. Depending on this temperature value, the heaters are either turned on or off.  

A GPIO output pin is used in conjunction with a MOSFET switching circuit, to supply an external power source to the heating pads, as the pads could not be powered by the microcontroller. 

A pushbutton is used to turn on and off the control, and an indicator is used to show if the system is operational or not.  

### KiCAD project files

The project files for this board can be found in "\PCB Design\Temperature Controller".

## Assembly and Testing of boards

## Other components needed

