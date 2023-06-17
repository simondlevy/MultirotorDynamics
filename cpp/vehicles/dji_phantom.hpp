/*
    Vehicle constants for DJI Phantom  

    Copyright (C) 2023 Simon D. Levy

    MIT License
*/

#pragma once

#include "../dynamics.hpp"

Dynamics::vehicle_params_t vparams = {

    // Estimated
    5.E-03,  // force constatnt b [F=b*w^2]
    2.E-06,  // drag coefficient d [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // mass m [kg]
    0.350,  // arm length L [m]

    // Estimated
    2,       // Ix [kg*m^2]
    2,       // Iy [kg*m^2]
    3,       // Iz [kg*m^2]
    38E-04,  // prop inertial Jr [kg*m^2]

    15000 // maxrpm
};

static FixedPitchDynamics::fixed_pitch_params_t fparams = {
    5.E-06, // b thrust coefficient [F=b*w^2]
    0.350   // l arm length [m]
};
