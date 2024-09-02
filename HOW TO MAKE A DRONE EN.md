# Teaching Ning How to Build a Drone

If there's any mistake, please contact [1107@siwg.top](mailto:1107@siwg.top). My abilities are limited.

This can be considered as a study record.

Please be careful operating with your drone!

It's recommended to use Typora for viewing, as it displays LaTeX formulas better.

You can buy all the stuff on Taobao cheaply, as the price would be usually higher on amazon or digikey(more recommended!)

Got a question but don’t know how to express it? Check this out! [Smart Questions Guide](https://github.com/1107-1108/smartquestions/blob/master/How%20to%20ask%20questions%20SMARTLY.md/)

## Disclaimer

This tutorial (Teaching Ning How to Build a Drone) (HOW TO MAKE A DRONE) provides all content for learning, communication, and sharing purposes only, for reference use. Readers may use the content provided in this tutorial for personal study, research, or appreciation, as well as other non-commercial or non-profit uses, but must comply with the open-source agreement and must not infringe upon the legitimate rights of the author of this tutorial and related rights holders.

This tutorial does not guarantee the completeness, accuracy, and timeliness of all content.

This tutorial contains third-party links for the convenience of the reader. The content of the links does not represent the views of this tutorial, and clicking on them may take you away from this tutorial to a third-party site. This tutorial is not responsible for any risks brought about by third-party sites.

Some operations in this tutorial involve safety hazards. Regardless of whether there is improper operation causing injury or death, this tutorial and its author are not responsible. Please operate cautiously.

By starting to read, you agree to the above disclaimer and assume all risks yourself.

The final interpretation of the content of this disclaimer, within the limits of the law, belongs to this tutorial and its author.

## Contributors

Author: 1107

Proofreader: Li3_Fish

## Outline

0. **Relevant Laws**
1. Material Preparation
2. Assembly Techniques
3. INS (Inertial Navigation System) Design

## Relevant Laws

Drone testing and flight must comply with local laws!!!

Please read the legal regulations carefully.

If you plan to bring drone batteries on a plane, please consult the airline in advance, as there may be restrictions.

### 0.1 Relevant Laws of the People's Republic of China

[Interim Regulations on the Flight Management of Unmanned Aerial Vehicles in the People's Republic of China](https://www.gov.cn/zhengce/content/202306/content_6888799.htm)

For example, I’m 17 years old, a minor (unfortunately, if I want to fly a drone in China, according to Chapter 2, Article 17):

> Chapter 2, Article 17 of the "Interim Regulations on the Flight Management of Unmanned Aerial Vehicles"
>
> Persons with no civil capacity can only operate micro civil unmanned aerial vehicles, and persons with limited civil capacity can only operate micro and light civil unmanned aerial vehicles. When a person with no civil capacity operates a micro civil unmanned aerial vehicle or a person with limited civil capacity operates a light civil unmanned aerial vehicle, it must be under the guidance of a person with full civil capacity present.

The drone I'm building falls under the category of light UAVs:

> Chapter 6, Article 62 of the "Interim Regulations on the Flight Management of Unmanned Aerial Vehicles":
>
> A light unmanned aerial vehicle refers to an unmanned aerial vehicle with an empty weight of no more than 4 kilograms and a maximum takeoff weight of no more than 7 kilograms, a maximum level flight speed of no more than 100 kilometers per hour, equipped with an airspace maintenance capability that meets airspace management requirements and reliable monitoring capability, and can be manually intervened at any time throughout the entire process, but does not include micro unmanned aerial vehicles.

And light UAVs:

> Chapter 2, Article 17 of the "Interim Regulations on the Flight Management of Unmanned Aerial Vehicles"
>
> Operators of micro and light civil unmanned aerial vehicles do not need to obtain an operator's license, but they must be proficient in operating the relevant model, be aware of risk warning information and related management systems.

So, to fly my drone in a **flyable area**, I first need to register my drone with real-name authentication at (UOM)[https://uom.caac.gov.cn/], and stick the real-name registration mark on the drone. Then, I need someone to supervise my flight.

### 0.2 United Kingdom

[Introduction to drone flying and the UK rules | Civil Aviation Authority (caa.co.uk)](https://www.caa.co.uk/drones/rules-and-categories-of-drone-flying/introduction-to-drone-flying-and-the-uk-rules/)

According to UK law, first, you need to follow these rules:

> Key rules include:
>
> - Never fly more than 120m (400ft) above the surface
> - Always keep your drone or model aircraft in sight
> - Never fly in an airport’s flight restriction zone unless you have permission

**As I understand it**, my drone weighs more than 250 grams, so I need to register for an Operator ID and Flyer ID. The cost of registering for an Operator ID is £11, and you need to pass an online theory exam (it seems to be an open-book test, so it shouldn't be too difficult).

### 0.3 Other Countries

Please check the local laws and regulations online.

You can refer to [Global Flight Safety Regulations Collection (Continuously Updated) (dji.com)](https://support.dji.com/help/content?customId=zh-cn03400009861&spaceId=34&re=CN&lang=zh-CN)

## Material Preparation

### 1.0 Introduction

This chapter discusses the selection of materials and chips, aiming to choose affordable, cheap materials that meet the usage scenarios for assembly because ***I AM RUNNING OUT OF MONEY***.

### 1.1 Material Selection and Purchase

**Electronic Speed Controller (ESC)**

Why is an ESC needed? Because the ESC's function is to convert control signals into current to control the motor's speed.

How to read ESC parameters? The ESC will indicate the current it can provide.

**Brushless Motor**

How to read motor model numbers? The model numbers of brushless motors, such as 2212, 2808, etc., the first two digits represent the rotor diameter (thickness), and the last two digits represent the rotor height (height), *not the shell*. Height and diameter are proportional to motor power.

What is the KV value? KV is the no-load speed per minute corresponding to an applied voltage of 1V. The lower the KV, the greater the torque.

**Propeller**

How to read propeller model numbers? The model number is also a number, such as 1045..., the first two digits represent the diameter, and the last two digits represent the pitch angle of the blades.

There are also clockwise and counterclockwise propellers, so be sure to distinguish them.

--------

All the following materials can be purchased from Taobao:

Frame: F450-V2 DJI, unit price 40 RMB
Landing Gear: *4, 8.99 RMB

Motor: Xinxida A2212 1400KV, unit price 30 *4 RMB

ESC: SKYWALKER 40A V2-UBEC, unit price 67 *4

Six-axis Attitude Sensor: MPU6050, unit price 60

Barometer: GY-63, unit price 21.8

Main Controller: Raspberry Pi Zero2W, unit price 143

Signal Transmission Module: DL-43P 2.4G, unit price 46.8 *2

HC-14, unit price 31.8 *2

Propellers: 1147, unit price 12*3

BB Sounder

**~~Actually not that cheap~~**

### 1.2 Information Transmission Module

For now, we are choosing LORA; more will be added later.

## Starting Assembly

### 2.0 Introduction

This chapter discusses the tools and technical support required to assemble a drone.

### 2.1 Tools

A set of various screwdrivers (especially hex and Phillips screwdrivers)

Needle-nose pliers, wire cutters

Soldering iron, solder, rosin (or other flux)

A thin, strong stick

Two boxes plus one long and thick stick (not too thick, after all, you need to mount the drone on it)

*This is used to test the PID, as no one wants to end up in the hospital while testing the drone.

A multimeter

A BB sounder

### 2.2 Soldering

Generally, the sensors bought from Taobao need to be soldered by yourself. This is where the soldering iron becomes very useful. It’s best to choose a soldering iron with adjustable temperature, which is not expensive.

When soldering pins, place the soldering iron and solder near the pins. When you see the solder melt next to the pin, the soldering is done (it's not difficult, after all, there are many tutorials online).

Another thing that needs to be soldered is the power wires to the PCB board. If the power wires don’t have solder, you can use some flux (I use rosin).

*After soldering, you can test with a multimeter to prevent cold solder joints.

*After testing, you can use silicone rubber to seal it to prevent the wires from coming loose during flight and touching other contacts. It’s cheap anyway.

**~~Although my soldering looks like shit~~**

### 2.3 Assembling the Drone

No need to say much about tightening screws; you can use thread-locking glue



#### 2.3.1 Motor Installation

The frame and motor must be securely fixed.

You can use the tools mentioned earlier to tighten the motor with screws, *there is nothing special about it*.

#### 2.3.2 ESC Installation

There are two installation methods: one is to hang the ESC under the motor, and the other is to place the ESC in the center frame of the drone. If you want to save money on wires, the first method is recommended.

For convenience, I chose to place the ESC on the frame.

#### 2.3.3 Propeller Installation

Attention! Some propellers rotate clockwise (CW), and some rotate counterclockwise (CCW), so pay attention to the direction of the motor and propeller.

*The rotation direction of the motor can be modified by reversing the power supply cables.

### 2.4 Installing the Flight Controller

For a DIY drone, the flight controller (FC) generally requires a lot of soldering.

You can put the flight controller on a vibration damping plate and attach a foam pad to protect it.

The Raspberry Pi doesn't need much to secure it. Just ensure it doesn’t shake during flight, and use glue and some wire clips to secure it to the frame.

## INS (Inertial Navigation System) Design

### 3.0 Introduction

This chapter introduces the control theory required to design a drone. It includes attitude estimation, sensor fusion, and PID tuning.

The code I used is written in Python, mainly because the learning cost is low and it can directly call MPU6050's smbus.

### 3.1 Sensor Theory

TODO

### 3.2 Sensor Fusion

#### 3.2.1 Complementary Filter

TODO

#### 3.2.2 Kalman Filter

The Kalman filter is more complex and requires the establishment of a state equation and a measurement equation.

The Kalman filter has the advantage of noise robustness, but the disadvantage is the computational complexity.

For instance, consider the acceleration and angular velocity as states, and their respective measurements can be used in the Kalman filter for estimation.

