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
/**	
    @name main.cpp
	@brief An example of how to use mmg_sim.hpp and run MMG model
	@author Jonas Mahler
	@date 12.2022
*/

#include <mmg_sim.hpp>

int main()
{
    MMGSim exampleSim;
    MMGSim::parameterSim varS;
    MMGSim::resultMMG resN;
    parameterMMG constN;

    // Read in values from file
    constN = exampleSim.readMMG("./mmg_config.yaml");
    varS = exampleSim.readSimulation("./sim_config.yaml");
    
    // Values can also be accessed and modified from inside main function
    varS.initYawRate = 0.0;

    resN = exampleSim.runSim(constN,varS);
}