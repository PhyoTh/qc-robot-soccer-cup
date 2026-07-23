# miniAuto Getting-Ready Guide

Source: [Hiwonder miniAuto — Getting Ready](https://docs.hiwonder.com/projects/miniAuto/en/latest/docs/1.getting_ready.html)
(accessed July 22, 2026).

Hiwonder identifies the original tutorial as copyrighted material.

## Product overview

miniAuto is an Arduino-compatible educational robot based on an ATmega328
controller. Depending on the kit, its hardware includes an ESP32-S3 vision
module, a glowing ultrasonic sensor, a four-channel line-following sensor, a
multifunction expansion board, Bluetooth connectivity, mecanum wheels, and an
optional gripper. Expansion ports support additional sensors and custom
projects.

![Assembled Hiwonder miniAuto robot](assets/miniauto-getting-ready/overview/image2.webp)

## Safety and handling

- The kit contains small parts, conductive pieces, and sharp pins. Hiwonder
  specifies that it is unsuitable for children under 12; minors should work
  under adult supervision.
- Do not swallow small parts, press against sharp components, or touch powered
  electronics with metal objects.
- Do not push or otherwise force the robot to move while power is on, because
  doing so can damage its drive system or electronics.
- Keep batteries and the charger away from heat and liquids. Do not modify,
  solder, or tamper with either item.
- For long-term storage, fully charge the batteries, switch off and remove the
  battery holder as appropriate, and keep the batteries in a cool, dry place.

## Kit inventory

Both standard and advanced kits are documented as containing the following:

| Item | Included parts | Quantity listed |
| --- | --- | ---: |
| Chassis set | Main chassis, top cover, back cover, front cover | 1 set (4 pieces) |
| Vision module set | ESP32-S3 vision module and two camera brackets | 1 set (3 pieces) |
| Mecanum wheel set | Two A wheels, two B wheels, four couplings, four pan-head screws | 1 set |
| Motors | Drive motors | 4 |
| Controller | UNO R3 | 1 |
| Expansion board | UNO R3 expansion board | 1 |
| Communication | Bluetooth module | 1 |
| Sensors | Glowing ultrasonic sensor; four-channel line follower | 1 each |
| Power | Two 18650 cells, battery holder, and charger | 1 set |
| Cables | USB Type-C, USB Type-B, and an additional USB cable | 1 each |
| Tool | Screwdriver | 1 |
| Hardware | Fasteners, standoffs, and four-pin cables listed below | 1 bag |

The advanced kit additionally includes one preassembled gripper bracket. The
vendor's packing list describes the accessory bag as containing:

- 10 × M3 × 4 flat-head screws
- 10 × M3 × 25 flat-head screws
- 25 × M3 × 6 black round-head screws
- 10 × M2 × 6 black round-head screws
- M4 × 8 black round-head screws (6 in the standard list; 10 in the advanced list)
- 20 × M3 nuts
- 5 × M3 × 8 double-pass copper standoffs
- 5 × M4 × 30 double-pass copper standoffs
- 3 × M3 × 6+6 single-pass nylon standoffs
- 2 × 100 mm four-pin cables
- 2 × 200 mm four-pin cables

Before assembly, compare the physical kit with the vendor's illustrated packing
list. Packaging and component revisions may differ.

### Packing-list reference images

| No. | Component | Vendor image |
| ---: | --- | --- |
| 1 | Chassis set | ![Chassis set](assets/miniauto-getting-ready/packing-list/1.webp) |
| 2 | ESP32-S3 vision module | ![ESP32-S3 vision module](assets/miniauto-getting-ready/packing-list/2.webp) ![Camera mounting bracket view 1](assets/miniauto-getting-ready/packing-list/2.1.webp) ![Camera mounting bracket view 2](assets/miniauto-getting-ready/packing-list/2.2.webp) |
| 3 | Mecanum wheel set | ![Mecanum wheels and fittings](assets/miniauto-getting-ready/packing-list/3.webp) |
| 4 | Motors | ![Drive motor](assets/miniauto-getting-ready/packing-list/4.webp) |
| 5 | USB Type-C cable | ![USB Type-C cable](assets/miniauto-getting-ready/packing-list/5.webp) |
| 6 | UNO R3 | ![UNO R3 controller](assets/miniauto-getting-ready/packing-list/6.webp) |
| 7 | UNO R3 expansion board | ![UNO R3 expansion board](assets/miniauto-getting-ready/packing-list/7.webp) |
| 8 | USB Type-B cable | ![USB Type-B cable](assets/miniauto-getting-ready/packing-list/8.webp) |
| 9 | Bluetooth module | ![Bluetooth module](assets/miniauto-getting-ready/packing-list/9.webp) |
| 10 | Charger | ![18650 battery charger](assets/miniauto-getting-ready/packing-list/10.webp) |
| 11 | 18650 batteries | ![Two 18650 batteries](assets/miniauto-getting-ready/packing-list/11.webp) |
| 12 | USB cable | ![USB cable](assets/miniauto-getting-ready/packing-list/12.webp) |
| 13 | Battery holder | ![Battery holder](assets/miniauto-getting-ready/packing-list/13.webp) |
| 14 | Glowing ultrasonic sensor | ![Glowing ultrasonic sensor](assets/miniauto-getting-ready/packing-list/14.webp) |
| 15 | Four-channel line follower | ![Four-channel line-following sensor](assets/miniauto-getting-ready/packing-list/15.webp) |
| 16 | Screwdriver | ![Included screwdriver](assets/miniauto-getting-ready/packing-list/16.webp) |
| 17 | Accessory bags | ![Standard-kit accessory bag](assets/miniauto-getting-ready/packing-list/17.1.webp) ![Advanced-kit accessory bag](assets/miniauto-getting-ready/packing-list/17.2.webp) |
| 18 | Advanced-kit gripper | ![Preassembled gripper bracket](assets/miniauto-getting-ready/packing-list/18.webp) |

## Assembly and wiring

The official guide provides the following 14-step visual sequence. Check part
orientation, wheel placement, fastener choice, and cable routing carefully.

### Step 1

![miniAuto assembly step 1](assets/miniauto-getting-ready/assembly/01.jpg)

### Step 2

![miniAuto assembly step 2](assets/miniauto-getting-ready/assembly/02.jpg)

### Step 3

![miniAuto assembly step 3](assets/miniauto-getting-ready/assembly/03.jpg)

### Step 4

![miniAuto assembly step 4](assets/miniauto-getting-ready/assembly/04.png)

### Step 5

![miniAuto assembly step 5](assets/miniauto-getting-ready/assembly/05.webp)

### Step 6

![miniAuto assembly step 6](assets/miniauto-getting-ready/assembly/06.webp)

### Step 7

![miniAuto assembly step 7](assets/miniauto-getting-ready/assembly/07.png)

### Step 8

![miniAuto assembly step 8](assets/miniauto-getting-ready/assembly/08.jpg)

### Step 9

![miniAuto assembly step 9](assets/miniauto-getting-ready/assembly/09.jpg)

### Step 10

![miniAuto assembly step 10](assets/miniauto-getting-ready/assembly/10.png)

### Step 11

![miniAuto assembly step 11](assets/miniauto-getting-ready/assembly/11.jpg)

### Step 12

![miniAuto assembly step 12](assets/miniauto-getting-ready/assembly/12.jpg)

### Step 13

![miniAuto assembly step 13](assets/miniauto-getting-ready/assembly/13.jpg)

### Step 14

![miniAuto assembly step 14](assets/miniauto-getting-ready/assembly/14.webp)

### Wiring diagram

![miniAuto wiring diagram](assets/miniauto-getting-ready/assembly/14.1.png)

### Completed installation

![Completed miniAuto assembly](assets/miniauto-getting-ready/assembly/15.png)

Before applying power:

1. Confirm that each mecanum wheel is in the illustrated position and
   orientation.
2. Check that the controller, expansion board, camera, sensors, motors, and
   battery holder match the official wiring diagram.
3. Make sure no loose fastener or conductive object can short the electronics.
4. Raise the wheels clear of the work surface for the first powered test.

## Charging and battery installation

Lithium cells may arrive only partly charged for shipping. Hiwonder recommends
charging the two 18650 cells for about one hour before the first use.

1. With the battery holder switched off, place both cells in the supplied
   charger in the polarity shown by the charger and official manual. Never
   reverse the positive and negative terminals.
2. If a separate USB power adapter is needed, use a 5 V adapter rated for
   1–2 A.
3. During charging, the charger indicator is red. It changes to green when the
   cells are charged.
4. Disconnect the charger promptly after completion to avoid overcharging.
5. Install the charged cells in the battery holder with the correct polarity,
   connect the holder as shown in the wiring diagram, and then switch it on.

![Correct placement of the batteries in the charger](assets/miniauto-getting-ready/charging/image2.png)

![Battery holder switched on after battery installation](assets/miniauto-getting-ready/charging/image3.webp)

Use the supplied charger unless Hiwonder provides an approved alternative. Stop
using a cell or charger that is damaged, swollen, unusually hot, wet, or behaving
abnormally, and follow local rules for safe battery handling and disposal.

## Vendor notice

Hiwonder states that its hardware and software are supplied as-is, that the
manual may contain errors or omissions, and that features can change between
product revisions. Consult the current official documentation or Hiwonder
support when a component, specification, or diagram differs from the kit in
hand. Hiwonder also disclaims responsibility for loss, damage, or safety events
caused by ignoring its battery and usage guidance.
