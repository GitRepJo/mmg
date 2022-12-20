// Copyright (c) 2022 Jonas Mahler
	
// This file is part of mmg.

// mmg is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// mmg is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with mmg. If not, see <https://www.gnu.org/licenses/>. 

#ifndef MMG_SIM_HPP
#define MMG_SIM_HPP

#include <mmg_ode.hpp>

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include<tuple>
#include<string>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <boost/math/differentiation/finite_difference.hpp>


/* Read model and simulation parameters and run a MMG ship model simulation*/ 
class MMGSim {
public:

    /**
    * @brief Constructor
    * @param -
    */
    MMGSim();
    /**
    * @brief Deconstructor
    * @param -
    */
    ~MMGSim();

    /* Constants used for the simulation of MMGs ship model*/
    struct parameterSim
    {
        double initYaw = 0.0; // Initial yaw in degree
        double initX = 0.0; // Initial position of x in meter
        double initY = 0.0; // Initial position of y in meter
        
        double initYawRate = 0.0; // Initial yaw rate in degree/s
        double initXVelocity = 0.0; // Initial x velcoity in m/s
        double initYVelocity = 0.0; // Initial y velocity in m/s
        
        double step = 0.0; // Stepsize for integration
        double time = 0.0; // Time to end of integration
        bool terminal_output = true ; // Write result struct to terminal if true
    };
    
    /* Save the results of the simulation of MMGs ship model*/
    struct resultMMG
    {
        std::vector<double> time; // Time corresponding to the state
        std::vector<double> x_pos; // Position x in m two dimensional carthesian global coordinates
        std::vector<double> y_pos; // Position y in m two dimensional carthesian global coordinates
        std::vector<double> x_vel; // Velocity x in m/s two dimensional carthesian global coordinates
        std::vector<double> y_vel; // Velocity y in m/s two dimensional carthesian global coordinates
        std::vector<double> x_acc; // Acceleration x in m/s^2 two dimensional carthesian global coordinates
        std::vector<double> y_acc; // Acceleration y in m/s^2 two dimensional carthesian global coordinates
        std::vector<double> yaw; // Orientation in global coordinates relative to north in degree
        std::vector<double> yaw_rate; // Yaw rate in degree/s
        std::vector<double> yaw_acc;  // yaw rate acceleration in degree/s*s
    };

    /**
    * @brief Run the MMG ship model simulation by using a predefined ordinary differential equation
    * @param cN constMMG struct to save constants for the MMG ODE
    * @param vS varSim struct to save constants variables specific to the simulation
    * @return resultMMG struct with the results (actual yaw angle, position ...)
    */
    MMGSim::resultMMG runSim(parameterMMG cN,parameterSim vS);
    
    /**
    * @brief Read constants for the MMG ship model
    * @param MMGFile .yaml file that matches variables in constMMG struct
    * @return constMMG struct with constants for MMG
    * @details The constants will be used to set up the MMG ship model functor
    */
    parameterMMG readMMG(std::string MMGFile);
    
    /**
    * @brief Read variables for the simulation of MMGs ship model
    * @param simFile -yanl file that matches variables in varSim struct
    * @return varSim struct with variables for specific simulation
    * @details The variables will be used by boost integration function to set up the simulation
    */
    parameterSim readSimulation(std::string simFile);

private:
    /**
    * @brief Calculate the yaw angle based on the simulation parameters and the position
    * @param cN constMMG struct to save constants for the MMG ODE
    * @param vS varSim struct to save constants variables specific to the simulation
    * @param states yaw rate result of the simulation, in this case only a one dimensional state vector
    * @param times Corresponding time for the yaw rate (no constant delta t with adaptive intregration)
    * @return resultMMG struct with the results (actual yaw angle, position ...)
    */
    resultMMG calcResult(parameterSim vS, std::vector<std::array<double,3>> &states , std::vector< double > &times);

    /**
    * @brief Write the result of the simulation to the terminal 
    * @param res struct that contains the result of the simulation
    * @details The values will be rounded to 2 digits after the comma
    */
    void writeTerminal(MMGSim::resultMMG res);

};

#endif //MMG_SIM_HPP