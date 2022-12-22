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

#ifndef MMG_OBS_HPP
#define MMG_OBS_HPP

#include <eigen3/Eigen/Dense>

/**
* @brief Functor to save the result of an integration with boost e.g. adaptive integration
* @param states std::vector<std::vector<double>> intermediate and result state(s) of integration
* @param times std::vector<double> Time that corresponds to according state
*/
class SaveMMG
{   
    public:

    std::vector<std::array<double,3>> &m_states;
    std::vector<double> &m_times;

    // Use vector of arrays to extend state vector of observer if required
    SaveMMG(std::vector<std::array<double,3>> &states, std::vector< double > &times ) : m_states( states ), m_times( times ) { }

    void operator()( const std::array<double,3> &x, const double t )
    {
        m_states.push_back(x);
        m_times.push_back(t);
    }
};

#endif //MMG_OBS_HPP