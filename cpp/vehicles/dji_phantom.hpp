/*
    Vehicle constants for DJI Phantom  

    Copyright (C) 2023 Simon D. Levy

    MIT License
*/

#pragma once

#include "../dynamics.hpp"

Dynamics::vehicle_params_t vehicle_params = {

    // Estimated
    5.E-03,  // force constatnt b [F=b*w^2]
    2.E-06,  // torque constant d [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // mass m [kg]

    // Estimated
    2,       // Ix [kg*m^2]
    2,       // Iy [kg*m^2]
    3,       // Iz [kg*m^2]
    38E-04,  // prop inertial Jr [kg*m^2]

    15000 // maxrpm
};
