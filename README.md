# Robotic-arm

## Overview

**[IMAGE]**
</br>
- Robotic arms are used extensively in several industries. They are electro-mechanical systems that replicate the articulated motion of human arms. The one developed here is for limb replacement. The movement of the 2 linked parts (upper arm and forearm) is predominantly through the actuated elbow joint. A DC motor controller in this project will drive this.
- The actuator is connected to the upper arm. For this project, we assumed the upper arm is fixed at a specific angle (5°). The actuator rotates the forearm employing a set of gears.
</br>
**[IMAGE]**
</br>
- The motion of the arm is regulated by an automatic control system that determines the necessary rotational deflection of the forearm. To achieve this, the arm must be equipped with the necessary system to ensure its automatic movement within its operating environment. The general principle of automated actuator systems is to feed information from joint rotation sensors to the arm’s control system.
- This system changes the voltage applied to the elbow actuator to produce the required rotational motion and thus change the deflection angle of the forearm.
- The elbow control system produces the required actuator rotation to move the forearm to a reference angle. It achieves this by comparing the actuator deflection angle θM (radians) with the reference angle θFref (radians). This indirectly controls the forearm’s deflection angle θF (radians).
- The Elbow control system uses the error difference between the reference deflection angle and the actuator’s deflection angle. In this case, the value for θFref (reference angle) is 55°. The reference deflection passes through the Reference Amp, represented by a simple gain KR. Also, the motor deflection is measured by the Actuator Sensor, which is represented by a simple gain KS.
- The compensated voltage, VA, is used to control the elbow actuator to drive the gears and thus indirectly generate an appropriate forearm deflection θF. it achieved this using a proportional gain GC and an integral term with gain K1 (assumed to be z in the initial stages of the project). These gains determine the performance of the control system.
- A key part of the overall Elbow Control System is the arm’s interaction with the actuator and gears. The actuator voltage is used to drive the actuator (i.e DC motor) to deflection θM (radians) using its generated torque TM (Nm). The drive shaft of the actuator is connected to Gear 1 (radius r1 meters), which acts as a load on the motor. As Gear 1 rotates, it transfers torque TF (Nm) to Gear 2 (radius r2 meters), which rotates the forearm to the desired deflection angle. This results in a gear ratio GR = r2/r1 and the torque TF = GRxTM.


## States and State Derivatives

| States | State Derivatives | Description |
|--------|-------------------|-------------|
| x1 = i | x1dot = 1/L * (Va-R * x1 - Ke * x3); | Current equation |
| x2 = θM | x2dot = x3 | Angle of the motor |
| x3 = θMdot | x3dot = 1/Jm * (Kt*x1-Bsm*(x3-x5)) | speed of the motor |
| x4 = θG1 | x4dot = x5 | angle of grear 1 |
| x5 = θG1dot | x5dot = 1/Jgear1 * (Bsm*(x3-x5)) | speed of gear 1 |
| x6 = θF | x6dot = x7 | angle of forearm/gear 2 |
| x7 = θFdot | x7dot = 1/Jf * ((GR * Kf * x4) - (Bsf * x7) - </br> ((Mf * Lf * g * sin(theta_U + x6)) / 2)); | speed of forearm |

## Constants

| Constant | Value | Unit | Description |
|----------|-------|------|-------------|
| Bsm | 0.01 | Nm/rad/s | damping coefficient for the motor |
| Bsf | 1.5 | Nm/rad/s | damping coefficient for the forearm |
| Jm | 0.002 | kgm2 | moment of inertia for the motor armature |
| Jgear1 | 0.001 | kgm2 | moment of inertia for grear 1 |
| Jf | 0.0204 | kgm2 | total moment inertia of the forearm and hand |
| Gc | 9.95 | N/A | elbow controller gain |
| GR | 1.7 | N/A | ratio between raduis of gear 1 and gear 2 |
| Ke | 0.35 | V/rad/s | back emf constant |
| Kf | 0.5 | N/A | torque gain |
| Kg | 2.0 | N/A | gear compensator gain |
| Kr | 0.9 | N/A | reference amp gain |
| Ks | 0.9 | N/A | motor deflection actuator sensor gain |
| Kt | 0.35 | Nm/A | torque constant |
| L | 0.1 | H | inductance |
| Lf | 0.35 | m | length of forearm |
| Mf | 0.5 | kg | mass of forearm |
| R | 4 | Ω | resistance |
| G | 9.81 | m/s<sup>2</sup> | acceleration of gravity |

## Responses

1. MATLAB reponse:
**[IMAGE]**
2. Simulink block diagram response:
**[IMAGE]**

The responses show how the motor angle, arm position and speed are related together. Initially, the motor changes its angle fast because its sensor shows that the arm is positioned at -9°. Therefore, the forearm’s speed and the angle changed quickly. Then, as the motor slows down, the speed and angle of the forearm slow down too until reaching a steady state at around 55° (i.e. the reference angle) and speed of 0.

## Initial Conditions, Numerical Integration Solver & Step-Size Selection

- Initial conditions: angle of forearm = theta F = -9°.
- Numerical Integration Solver: Runge-Kutta 4th order:
  - This provides us with sufficient accuracy and computation time for this application. Runge-Kutta 4th order has a truncation error of the 5th term of the Taylor series. The 5th term of the Taylor series of almost any system will be very close to zero, resulting in an accurate numerical integration solver.
- Step size = 0.001s:
  - After calculating the time constant of the system, I found out that any step-size less than 0.0025 will be sufficient. Therefore, 0.001 has been selected for more accurate measurements.

## Block diagram
overview:
**[IMAGE]**

1. All system:
**[IMAGE]**
2. Actuator:
**[IMAGE]**
3. Gear system:
overview
**[IMAGE]**
implementation
**[IMAGE]**
1. Forearm dynamic:
**[IMAGE]**

## Results
**[IMAGE]** 

