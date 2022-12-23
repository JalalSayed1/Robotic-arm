# Robotic Arm

## Overview

![Prosthetic Arm Elbow Motion Schematic Diagram](https://user-images.githubusercontent.com/92950538/202859147-3f3b5eab-64fc-41b0-ac0b-6f2496a497b9.png)
</br>

- Robotic arms are used extensively in several industries. They are electro-mechanical systems that replicate the articulated motion of human arms. The one developed here is for limb replacement. The movement of the 2 linked parts (upper arm and forearm) is predominantly through the actuated elbow joint. A DC motor controller in this project will drive this.
- The actuator is connected to the upper arm. For this project, we assumed the upper arm is fixed at a specific angle (5°). The actuator rotates the forearm employing a set of gears.

![Geometry of rotating motion](https://user-images.githubusercontent.com/92950538/202859195-daed0df4-660d-497c-8ba3-9f548bdf2c70.png)
</br>

- The motion of the arm is regulated by an automatic control system that determines the necessary rotational deflection of the forearm. To achieve this, the arm must be equipped with the necessary system to ensure its automatic movement within its operating environment. The general principle of automated actuator systems is to feed information from joint rotation sensors to the arm’s control system.
- This system changes the voltage applied to the elbow actuator to produce the required rotational motion and thus change the deflection angle of the forearm.
- The elbow control system produces the required actuator rotation to move the forearm to a reference angle. It achieves this by comparing the actuator deflection angle θM (radians) with the reference angle θFref (radians). This indirectly controls the forearm’s deflection angle θF (radians).
- The Elbow control system uses the error difference between the reference deflection angle and the actuator’s deflection angle. In this case, the value for $\theta$<sub>Fref</sub> (reference angle) is 55°. The reference deflection passes through the Reference Amp, represented by a simple gain KR. Also, the motor deflection is measured by the Actuator Sensor, which is represented by a simple gain KS.
- The compensated voltage, VA, is used to control the elbow actuator to drive the gears and thus indirectly generate an appropriate forearm deflection $\theta$<sub>F</sub>. It achieved this using a proportional gain GC and an integral term with gain K1 (assumed to be z in the initial stages of the project). These gains determine the performance of the control system.
- A key part of the overall Elbow Control System is the arm’s interaction with the actuator and gears. The actuator voltage is used to drive the actuator (i.e DC motor) to deflection θM (radians) using its generated torque TM (Nm). The drive shaft of the actuator is connected to Gear 1 (radius r1 meters), which acts as a load on the motor. As Gear 1 rotates, it transfers torque TF (Nm) to Gear 2 (radius r2 meters), which rotates the forearm to the desired deflection angle. This results in a gear ratio GR = r2/r1 and the torque TF = GRxTM.


## States and State Derivatives

| States | State Derivatives | Description |
|--------|-------------------|-------------|
| $x$<sub>1</sub> = i | $\dot{x}$<sub>1</sub> = 1/L * (Va-R * x<sub>1</sub> - Ke * x3); | Current equation |
| $x$<sub>2</sub> = θ<sub>M</sub> | $\dot{x}$<sub>2</sub> = x<sub>3</sub> | Angle of the motor |
| $x$<sub>3</sub> = θ<sub>Mdot</sub> | $\dot{x}$<sub>3</sub> = 1/Jm * (Kt*x1-Bsm*(x<sub>3</sub>-x<sub>5</sub>)) | Speed of the motor |
| $x$<sub>4</sub> = θ<sub>G1</sub> | $\dot{x}$<sub>4</sub> = x<sub>5</sub> | Angle of grear 1 |
| $x$<sub>5</sub> = θ<sub>G1dot</sub> | $\dot{x}$<sub>5</sub> = 1/Jgear1 * (Bsm*(x<sub>3</sub>-x<sub>5</sub>)) | Speed of gear 1 |
| $x$<sub>6</sub> = θ<sub>F</sub> | $\dot{x}$<sub>6</sub> = x<sub>7</sub> | Angle of forearm/gear 2 |
| $x$<sub>7</sub> = θ<sub>Fdot</sub> | $\dot{x}$<sub>7</sub> = 1/Jf * ((GR * Kf * x<sub>4</sub>) - (Bsf * x<sub>7</sub>) - ((Mf * Lf * g * sin(theta_U + x<sub>6</sub>)) / 2)); | Speed of forearm |

## Constants

| Constant | Value | Unit | Description |
|----------|-------|------|-------------|
| Bsm | 0.01 | Nm/rad/s | Damping coefficient for the motor |
| Bsf | 1.5 | Nm/rad/s | Damping coefficient for the forearm |
| Jm | 0.002 | kgm2 | Moment of inertia for the motor armature |
| Jgear1 | 0.001 | kgm2 | Moment of inertia for gear 1 |
| Jf | 0.0204 | kgm2 | Total moment inertia of the forearm and hand |
| Gc | 9.95 | N/A | Elbow controller gain |
| GR | 1.7 | N/A | Ratio between radius of gear 1 and gear 2 |
| Ke | 0.35 | V/rad/s | Back emf constant |
| Kf | 0.5 | N/A | Torque gain |
| Kg | 2.0 | N/A | Gear compensator gain |
| Kr | 0.9 | N/A | Reference amp gain |
| Ks | 0.9 | N/A | Motor deflection actuator sensor gain |
| Kt | 0.35 | Nm/A | Torque constant |
| L | 0.1 | H | Inductance |
| Lf | 0.35 | m | Length of forearm |
| Mf | 0.5 | kg | Mass of forearm |
| R | 4 | Ω | Resistance |
| G | 9.81 | m/s<sup>2</sup> | Acceleration of gravity |

## Initial Conditions, Numerical Integration Solver & Step-Size Selection

- Initial conditions: angle of forearm = theta F = -9°.
- Numerical Integration Solver: Runge-Kutta 4th order:
  - This provides us with sufficient accuracy and computation time for this application. Runge-Kutta 4th order has a truncation error of the 5th term of the Taylor series. The 5th term of the Taylor series of almost any system will be very close to zero, resulting in an accurate numerical integration solver.
- Step size = 0.001s:
  - After calculating the time constant of the system, I found out that any step-size less than 0.0025 will be sufficient. Therefore, 0.001 has been selected for more accurate measurements.

## Block diagram
1. Overview:
![image](https://user-images.githubusercontent.com/92950538/202859319-eb0b06ce-26ce-404b-b500-762f08a2036f.png)
2. All system:
![image](https://user-images.githubusercontent.com/92950538/202859299-3bb8568a-e163-4c38-bf11-8d2e0daffff0.png)
3. Actuator:
![image](https://user-images.githubusercontent.com/92950538/202859303-d59dcecf-1003-456f-8f87-fb6e82e0c4c9.png)
4. Gear system:
4.1. Overview:
![image](https://user-images.githubusercontent.com/92950538/202859329-c85735b1-50d1-4f7d-82c6-df69a12d024b.png)
4.2. Implementation:
![image](https://user-images.githubusercontent.com/92950538/202859306-fac35497-b2a1-4f01-8904-5e3d636ef8b0.png)
5. Forearm dynamic:
![image](https://user-images.githubusercontent.com/92950538/202859341-43680292-f4ec-4f2e-9592-19294e17d293.png)

## System Response #1

1. MATLAB reponse:
![MATLAB reponse](https://user-images.githubusercontent.com/92950538/202859243-9af1a79e-61b9-4c27-979a-727c6956b989.png)

2. Simulink block diagram response:
![Simulink block diagram response](https://user-images.githubusercontent.com/92950538/202859258-ed1988d7-fae9-499a-8a69-6ca4720e363b.png)
</br>

- The responses show how the motor angle, arm position and speed are related together. Initially, the motor changes its angle fast because its sensor shows that the arm is positioned at -9°. Therefore, the forearm’s speed and the angle changed quickly. Then, as the motor slows down, the speed and angle of the forearm slow down too until reaching a steady state at around 55° (i.e. the reference angle) and speed of 0.

## Better controller

- Previous results shows how wasn't really good for this system (i.e. very oscillatory). The reason for this was that the design of the controller was not suitable.
- key parameter is the upper arm deflection angle,  $θ$<sub>U</sub>. This deflection represents the motion of the upper arm and the variation in this value represents approximate changes in the dynamics of the upper arm.
- In this part, the variation in values is achieved by **interpolating** data points that represent the upper angle deflection at specific time points.
- Therefore, Ve equation will be changed:
  - From simple Gain $Ve = Gc * \Deltaθ$
  - To $Ve = Gc*(1+Ki/s) \Deltaθ$ ; where `1/s` is the Laplace operator and `Ki` is the integral gain.

Using the mentioned equation, we will be varying two variables: Gc (the compensator gain) and Ki (the integral gain) and investigate how changing these two parameters can improve the performance of our system.

Then, to vary the upper arm angle over time, we will use the Quadratic Splines interpolation method to estimate its value using lab data obtained over short time intervals:

| Time (s) | 0 | 3.5 | 7.5 | 16.5 | 21.5 | 25 |
|---|---|---|---|---|---|---|
| $\theta$u | 5.0 | 10.5 | 24.5 | 27.5 | 35.5 | 45.5 |

## Gc Control Gain Variation Results & Analysis

![image](https://user-images.githubusercontent.com/92950538/207927402-cfac2f9d-f790-48ec-9ab5-32b8f34d2c45.png)



## KI Control Gain Variation Results & Analysis
As we found previously that Gc = 0.5 will produce the best results, we will fix this value here. Now, by varying Ki from 0 to 5, we will notice that above 4, the graph will start to wobble. This means that the value of Ki = 4 is better than any of the values above this one. Adding the integrator gain leads to some initial damping (i.e., around 0.6s). However, it makes the system reaches a steady-state much faster than without it. For instance, with Ki = 0 (i.e. no integrator gain), the system reaches steady-state at about 1.8s. Adding the integrator gain value of 4 leads the system to reach this steady state at about 1s.

![image](https://user-images.githubusercontent.com/92950538/207927743-4fe2e149-bc7b-445b-9738-77463ed6fdf1.png)
