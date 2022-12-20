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


#include <mmg_sim.hpp>
#include <mmg_obs.hpp>

MMGSim::MMGSim()
{}

MMGSim::~MMGSim()
{}

MMGSim::resultMMG MMGSim::runSim(parameterMMG cN,MMGSim::parameterSim vS)
{
    typedef boost::numeric::odeint::runge_kutta_dopri5< std::array< double,3 >  > stepper_type;
    
    std::array<double,3> start_state1 = {vS.initXVelocity, vS.initYVelocity,vS.initYawRate};
    
    MMGOde sim(cN);
    
    std::vector<std::array<double,3>> m_states;
    std::vector<double> m_times;

    SaveMMG observe(m_states, m_times);
    
    std::cout << "running odeint \n"; 
    boost::numeric::odeint::integrate_adaptive( make_controlled( 1E-12 , 1E-12 , stepper_type() ),
                        sim , start_state1 , 0.0 , vS.time , vS.step , observe );

    std::cout << "odeint finished. \n"; 
    resultMMG res = calcResult(vS,m_states,m_times);

    if (vS.terminal_output == true)
    {
        writeTerminal(res);
    }  

    return res;
}

MMGSim::resultMMG MMGSim::calcResult(parameterSim vS, std::vector<std::array<double,3>> &states , std::vector< double > &times)
{
    MMGSim::resultMMG res;
    
    res.x_pos.push_back(vS.initX);
    res.y_pos.push_back(vS.initY);
    res.yaw.push_back(vS.initYaw);
    
    res.yaw_rate.push_back(vS.initYawRate);
    res.x_vel.push_back(vS.initXVelocity);
    res.y_vel.push_back(vS.initYVelocity);
    
    res.time.push_back(0);
    res.yaw_acc.push_back(0);
    res.x_acc.push_back(0);
    res.y_acc.push_back(0);

    std::vector<double> delta_time;

    for (std::vector<double>::size_type i = 1; i < states.size(); i++)
    {
        // Access first array element of vector of arrays "states" at position i. In this implementation, the 
        // array is of size 3 because the state array is of size 3 (only yaw rate).
        res.yaw_rate.push_back(states.at(i)[3]);
        res.x_vel.push_back(states.at(i)[1]);
        res.y_vel.push_back(states.at(i)[2]);

        res.time.push_back(times.at(i));

        double delta_time = res.time.at(i) - res.time.at(i-1);
        
        // yaw = (yaw rate of step * time of step) + previous yaw
        double yaw = res.yaw_rate.at(i) * delta_time + res.yaw.at(i-1);
        res.yaw.push_back(yaw);

         // yaw acceleration = (yaw rate of step - yaw rate of previous step) * time of step
        double yaw_acc = (res.yaw_rate.at(i) - res.yaw_rate.at(i-1)) * delta_time;
        res.yaw_acc.push_back(yaw_acc);
      
        // Position in x
        double x_pos = res.x_pos.at(i-1) + res.x_vel.at(i) * delta_time; 
        res.x_pos.push_back(x_pos); 

        // Position in y 
        double y_pos = res.y_pos.at(i-1) + res.y_vel.at(i) * delta_time; 
        res.y_pos.push_back(y_pos); 
    }

    return res;
}

MMGSim::parameterSim MMGSim::readSimulation(std::string simFile)
{
    MMGSim::parameterSim var;

    YAML::Node config = YAML::LoadFile(simFile);

    var.step    = config["step"].as<double>(); 
    var.time    = config["time"].as<double>(); 
    
    var.initYaw = config["initYaw"].as<double>(); 
    var.initX = config["initX"].as<double>(); 
    var.initY = config["initY"].as<double>(); 

    var.initYawRate = config["initYawRate"].as<double>(); 
    var.initXVelocity= config["initXVelocity"].as<double>();
    var.initYVelocity= config["initYVelocity"].as<double>();

    var.terminal_output = config["terminal_output"].as<bool>();

    return var;
}

parameterMMG MMGSim::readMMG(std::string mmgFile)
{
    parameterMMG pa;
    
    YAML::Node config = YAML::LoadFile(mmgFile);
	
    std::cout << "Start reading\n"<< std::endl;
	pa.delta  	      = config["delta"].as<double>();  
    pa.npm 	          = config["npm"].as<double>();    
    
    pa.roh  	      = config["roh"].as<double>();  
    pa.L_pp 	      = config["L_pp"].as<double>();    
    pa.B    	      = config["B"].as<double>();   
    pa.d    	      = config["d"].as<double>();     
    pa.nabla	      = config["nabla"].as<double>();  
    pa.x_G  	      = config["x_G"].as<double>();   
    pa.C_b  	      = config["C_b"].as<double>();  
    pa.D_p  	      = config["D_p"].as<double>(); 
    pa.H_R  	      = config["H_R"].as<double>(); 
    pa.A_R  	      = config["A_R"].as<double>();  
    pa.t_P  	      = config["t_P"].as<double>();  
    pa.w_P0 	      = config["w_P0"].as<double>(); 
    pa.m_x_dash    = config["m_x_dash"].as<double>();  
    pa.m_y_dash    = config["m_y_dash"].as<double>();  
    pa.J_z_dash    = config["J_z_dash"].as<double>();  
    pa.t_R         = config["t_R"].as<double>(); 
    pa.x_R_dash    = config["x_R_dash"].as<double>(); 
    pa.a_H         = config["a_H"].as<double>();  
    pa.x_H_dash    = config["x_H_dash"].as<double>();  
    pa.Y_R_minus   = config["Y_R_minus"].as<double>();  
    pa.Y_R_plus    = config["Y_R_plus"].as<double>();  
    pa.l_r_dash    = config["l_r_dash"].as<double>();  
    pa.x_P_dash    = config["x_P_dash"].as<double>();  
    pa.epsilon           = config["epsilon"].as<double>();  
    pa.kappa           = config["kappa"].as<double>();    
    pa.f_alpha         = config["f_alpha"].as<double>();   
    
    std::cout << "Basic parameters are read sucessfully.\n"<< std::endl;
	
    pa.k_0         = config["k_0"].as<double>();
    pa.k_1         = config["k_1"].as<double>();
    pa.k_2         = config["k_2"].as<double>();
    pa.R_0_dash    = config["R_0_dash"].as<double>();
    pa.X_vv_dash   = config["X_vv_dash"].as<double>();
    pa.X_vr_dash   = config["X_vr_dash"].as<double>();
    pa.X_rr_dash   = config["X_rr_dash"].as<double>();
    pa.X_vvvv_dash = config["X_vvvv_dash"].as<double>();
    pa.Y_v_dash    = config["Y_v_dash"].as<double>();
    pa.Y_r_dash    = config["Y_r_dash"].as<double>();
    pa.Y_vvv_dash  = config["Y_vvv_dash"].as<double>();
    pa.Y_vvr_dash  = config["Y_vvr_dash"].as<double>();
    pa.Y_vrr_dash  = config["Y_vrr_dash"].as<double>();
    pa.Y_rrr_dash  = config["Y_rrr_dash"].as<double>();
    pa.N_v_dash    = config["N_v_dash"].as<double>();
    pa.N_r_dash    = config["N_r_dash"].as<double>();
    pa.N_vvv_dash  = config["N_vvv_dash"].as<double>();
    pa.N_vvr_dash  = config["N_vvr_dash"].as<double>();
    pa.N_vrr_dash  = config["N_vrr_dash"].as<double>();
    pa.N_rrr_dash  = config["N_rrr_dash"].as<double>();
	
    std::cout << "Maneuvering parameters are read sucessfully.\n"<< std::endl;
	
    pa.m   = pa.roh * pa.nabla;
	pa.I_zG= pa.roh * pa.nabla * (pow((0.25 * pa.L_pp), 2));  
    pa.eta   = pa.D_p / pa.H_R,  
    pa.m_x = (0.5 * pa.roh * (pow(pa.L_pp, 2)) * pa.d) * pa.m_x_dash;  
    pa.m_y = (0.5 * pa.roh * (pow(pa.L_pp, 2)) * pa.d) * pa.m_y_dash;  
    pa.J_z = (0.5 * pa.roh * (pow(pa.L_pp, 4)) * pa.d) * pa.J_z_dash;  
    pa.x_R = pa.x_R_dash * pa.L_pp;  
    pa.x_H = pa.x_H_dash * pa.L_pp; 

    return pa;
}

void MMGSim::writeTerminal(MMGSim::resultMMG res)
{
    for (std::vector<double>::size_type i = 0; i < res.time.size(); i++)
        {
            double t = round(res.time.at(i)  *100)/100;
            double x = round(res.x_pos.at(i) *100)/100;
            double y = round(res.y_pos.at(i) *100)/100;
            double yaw = round(res.yaw.at(i) *100)/100;
            
            std::cout <<"t[sec]: "<< t <<" x[m]: "<< x <<" y[m]: "<< y <<" yaw[deg]: " << yaw << '\n' << "\n";
        } 
}