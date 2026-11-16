# Automated Biaxial Stretching Mechanism for Dielectric Elastomers

A B.Tech project at **IIT Indore** focused on the design and development of a compact, automated mechanism for applying independently controlled biaxial pre-stretch to dielectric elastomer (DE) specimens.

> **Project status:** The end-evaluation presentation documents mechanical design, CAD development, 3D-printed prototype work, hardware selection and planning, initial Arduino UNO interface code, a partial Arduino MEGA motion-control framework, and UNO–MEGA communication. Hardware testing, calibration, final integration and model validation are listed as future work. This README does not claim that full-system performance has been experimentally validated.



## Table of Contents
- [Motivation](#motivation)
- [Objectives](#project-objectives)
- [Mechanism Overview](#mechanism-overview)
- [Design Development](#design-development)
- [Key Design Parameters](#key-design-parameters)
- [Electronics and Control Architecture](#electronics-and-control-architecture)
- [User Interface](#user-interface-and-operating-concept)
- [Work Completed](#work-completed)
- [Future Work](#future-work)
- [Project Team](#project-team)

## Motivation

Dielectric elastomers are soft electroactive materials capable of large deformation and electromechanical coupling. They are used in actuators, sensors, energy harvesters, haptic devices and soft robotics.

A key challenge in dielectric elastomer actuation is electromechanical instability (EMI). Mechanical pre-stretch can help stabilize actuation and improve performance. This project addresses the need for a mechanism that can apply controlled biaxial pre-stretch while maintaining specimen orientation.

## Project Objectives

The objective is to develop a fully automated biaxial stretching mechanism for dielectric elastomers, with these design goals:

- Motorized actuation with position feedback and automated positioning.
- Independent X–Y biaxial stretch control.
- Direct user input of stretch ratios through a simple screen-based interface.
- Compact, integrated and low-cost construction using 3D-printed components and electronics.
- Compliant fixtures intended to preserve specimen orientation and promote uniform stretching.
- Self-locking position holding for longer-duration pre-stretch without unnecessary continuous power.
- Target stretch-ratio input resolution of **0.01×**.

## Mechanism Overview

The mechanism uses a rack-and-pinion-based transmission to generate motion along two perpendicular axes:

- **X-direction:** A source gear drives a translating T-rack/carrier assembly.
- **Y-direction:** Two central gears, mounted on the base and moving carrier, drive four cross-oriented L-racks.
- **Specimen gripping:** An eight-pin scissor arrangement acts as a compliant fixture between the mechanism and the dielectric elastomer.

The scissor fixture is intended to distribute displacement around the specimen while helping maintain its orientation during biaxial stretching.


### Eight-pin scissor fixture

The final concept uses an eight-pin arrangement connected through a scissor mechanism. The presentation identifies reduced stress concentration, improved stress distribution and a compliant interface to the specimen as intended advantages.

### Exploded assembly

The assembly is organized around the lower base, X-direction T-base and T-rack, central gears and motors, cross L-racks, and the scissor mechanism with the DE specimen.

### Prototype

The project presentation includes the following photograph of the 3D-printed mechanism.


## Design Development

The mechanical design was developed in **Autodesk Fusion 360**. The documented workflow included:

1. Developing the design concept and measuring components.
2. Creating 3D models of the mechanical parts.
3. Assembling components and checking motion in Fusion 360.
4. Reviewing clearances and tolerances and making design corrections.
5. 3D printing components to check fit and functionality.

The design evolved from earlier concepts to an eight-pin scissor-based fixture. The final design was intended to reduce stress concentration, improve load distribution across the elastomer and simplify mechanical constraints and hardware integration.

## Key Design Parameters

The following values are listed in the project presentation.

| Component / parameter | Value |
|---|---:|
| Gear module | 2 |
| Gear pressure angle | 20° |
| Gear tooth count | 28 |
| Gear pitch radius | 14 mm |
| Gear thickness | 28 mm |
| T-rack module | 2 |
| T-rack tooth count | 66 |
| T-rack length | 419.44 mm |
| T-rack width | 50 mm |
| T-rack thickness | 30 mm |
| Cross L-rack module | 2 |
| Cross L-rack tooth count | 63 |
| Cross L-rack length | 400.6 mm |
| Cross L-rack width | 20 mm |
| Cross L-rack thickness | 30 mm |
| T-base length | 410 mm |
| T-base width | 238 mm |
| Rack teeth to walls clearance | 0.4 mm |
| Clearance between two walls | 0.4 mm |
| Rack teeth to gear teeth clearance | 0.15 mm |


## Electronics and Control Architecture

The proposed system combines a user-interface controller and a motion-control controller.

### Hardware listed in the project

- Arduino UNO and Arduino MEGA microcontrollers.
- DC worm-geared motors with encoders.
- Motor-driver modules.
- Normally open limit switches.
- 12 V SMPS power supply and 5 V buck converter.
- LCD display, rotary encoder and Back button.
- Terminal blocks, fuse and power switch.

### Control concept

- **UNO:** Intended to manage the display, menu navigation and user inputs.
- **MEGA:** Intended to manage motor actuation, encoder feedback, homing and positioning.
- **Communication:** The controllers exchange commands and status information over a serial link.
- **Feedback:** Encoder signals provide motor-output position information.
- **Homing:** Limit switches provide the home-position reference.

### Circuit diagram

The following circuit diagram is reproduced from the project presentation as the proposed circuit arrangement.

## User Interface and Operating Concept

The planned interface uses a character LCD, a KY-040 rotary encoder and a separate momentary Back button.

The intended workflow is:

1. Display a welcome/initialization screen.
2. Select **STRETCH** or **HOME** from the main menu.
3. Enter independent X- and Y-direction stretch ratios.
4. Confirm the selected values and initiate the stretching sequence.
5. Use the home function to return the mechanism to its reference position.

The intended input range is **1.00× to 5.00×**, with a resolution of **0.01×**.

## Work Completed

The project presentation records work in three areas.

### Mechanical design
- Design concept development and component measurements.
- 3D modelling of components in Autodesk Fusion 360.
- Assembly and motion simulation in Fusion 360.
- Tolerance review and design corrections.
- 3D printing of components to check fit and functionality.

### Hardware planning
- Review of hardware requirements.
- Selection and planning of electronic components.
- Signal-flow planning.
- Preparation of the circuit diagram.

### Automation
- Initial Arduino UNO code for the user interface, display and buttons.
- Partial Arduino MEGA framework for motion control.
- Establishment of UNO–MEGA serial communication.

## Final modifications

- Hardware testing.
- Motor and encoder calibration.
- Design modifications based on test results.
- Final part fabrication.
- Hardware integration and wiring.
- Model testing and rectification.

  ## List of Hardware components ordered

  https://onedrive.live.com/:x:/g/personal/432448a131883864/IQACBHqHHscxQoaJMehmzSOOAbpldP8N4flmG1kdr_zZOf4?rtime=U3fTJxtE30g&nav=MTVfezAwMDAwMDAwLTAwMDEtMDAwMC0wMDAwLTAwMDAwMDAwMDAwMH0&redeem=aHR0cHM6Ly8xZHJ2Lm1zL3gvYy80MzI0NDhhMTMxODgzODY0L0lRQUNCSHFISHNjeFFvYUpNZWhtelNPT0FicGxkUDhONGZsbUcxa2RyX3paT2Y0P2U9ZGM5WEswJm5hdj1NVFZmZXpBd01EQXdNREF3TFRBd01ERXRNREF3TUMwd01EQXdMVEF3TURBd01EQXdNREF3TUgw

