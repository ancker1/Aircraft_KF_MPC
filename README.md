# Aircraft Pitch Control

This project have been made as a part of the 5 ECTS course **Optimization and Control** at the Technical Faculty, University of Southern Denmark.
The project have been made by:
* **Emil Vincent Ancker**>	**emanc16@student.sdu.dk**
* **Jens Beltman Jørgensen**>	**jensj16@student.sdu.dk**
* **Søren Graabæk**>		**sogra16@student.sdu.dk**

## Aircraft_KF_MPC.m

The repository contains code for implementation of an output-feedback MPC for aicraft pitch control.

Kalman filter is used to provide an optimal estimate of the state.

Quadprog with a non zero reference input is used to make the aircraft reach a desired pitch angle.


