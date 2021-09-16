# A123_26650_MPCcharging

This repository contains the battery charging control algorithm for the A123 26650 m1b cell developed and used in the following publications

<a href="https://mountainscholar.org/handle/10976/167269">Kawakita de Souza, A. (2020). Advanced Predictive Control Strategies for Lithium-Ion Battery Management Using a Coupled Electro-Thermal Model [Master thesis, University of Colorado, Colorado Springs]. ProQuest Dissertations Publishing.</a>

<a href="https://ieeexplore.ieee.org/document/9124431">A. K. de Souza, G. Plett and M. S. Trimboli, "Lithium-Ion Battery Charging Control Using a Coupled Electro-Thermal Model and Model Predictive Control," 2020 IEEE Applied Power Electronics Conference and Exposition (APEC), 2020, pp. 3534-3539, doi: 10.1109/APEC39645.2020.9124431.</a>

- This software uses Model Predictive Control(MPC) to enforce constraints on current, voltage and temperature during charging. 
- The underlying model of this MPC-based algorithm is a coupled electro-thermal (CET) model developed in thesis above. The details of the CET model is presented below. The CET model can be parameterized using the <a href="https://data.mendeley.com/datasets/p8kf893yv3/1">A123 26650 dataset</a> . For details about the model parameterization see the <a href="https://mountainscholar.org/handle/10976/167269">thesis</a>.<br/>
- mainMPC.m is the main file to run the MPC algorithm
