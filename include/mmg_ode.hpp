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

#ifndef MMG_ODE_HPP
#define MMG_ODE_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;

/* Parameters used for MMG ship model*/
struct parameterMMG
{ 
    double delta; // Rudder angle [degree]
    double npm; // Propeller revolutions [rounds per minute]
    double roh; // Water density
    double L_pp;   // Ship length between perpendiculars [m]
    double B;   // Ship breadth[m]
    double d;   // Draft[m]
    double nabla; // Displacement[m^3]
    double x_G;   // Longitudinal coordinate of center of gravity of ship[m]
    double C_b;  // Block coefficient[-]
    double D_p; // Propeller diameter[m]
    double H_R; // Rudder span length[m]
    double A_R; // Profile area of movable part of mariner rudder[m^2]
    double t_P; // Thrust deduction factor
    double w_P0; // Wake coefficient at propeller position in straight moving
    double m_x_dash; // Added masses of x axis direction
    double m_y_dash; // Added masses of y axis direction
    double J_z_dash; // Added moment of inertia
    double t_R; // Steering resistance deduction factor
    double x_R_dash; // Relative position of the rudder
    double a_H; // Rudder force increase factor
    double x_H_dash; //    Steering force increment position
    double Y_R_minus; // Rectification coefficient
    double Y_R_plus; // Rectification coefficient
    double l_r_dash; // Rudder position relative to the captain
    double x_P_dash; // Propeller position relative to the captain
    double epsilon; // Ratio of wake fraction at propeller and rudder positions 
    double kappa;   // An experimental constant for expressing u R
    double f_alpha;  // Rudder lift gradient coefficient
    double m ; // Displacement[kg]
    double I_zG; // Moment of inertia
    double eta   ; // Rudder height relative to propeller diameter(Dp/H)
    double m_x ; // Attached mass x(No dimension)
    double m_y ; // Attached mass y(No dimension)
    double J_z ; // Addition Mass Izz(No dimension)
    double x_R ; // Relative position of the rudder
    double x_H ;  // Steering force increment position

    double k_0 ;
    double k_1 ;
    double k_2 ;
    double R_0_dash;
    double X_vv_dash ;
    double X_vr_dash;
    double X_rr_dash;
    double X_vvvv_dash;
    double Y_v_dash ;
    double Y_r_dash;
    double Y_vvv_dash ;
    double Y_vvr_dash;
    double Y_vrr_dash ;
    double Y_rrr_dash;
    double N_v_dash ;
    double N_r_dash ;
    double N_vvv_dash ;
    double N_vvr_dash ;
    double N_vrr_dash;
    double N_rrr_dash ;
};

/** 
* @brief Functor Ordinary Differential Equation for MMG ship model
* @param param struct of coefficients used by the mmg ordinary differential equation
* @details Equations follow the publication of H. Yasukawa • Y. Yoshimura 2014
* "Introduction of MMG standard method for ship maneuvering predictions" in JASNAOE
*/
class MMGOde {

public:

    const parameterMMG &pm;
    
    MMGOde( parameterMMG &param) :  pm(param) { };

    // Use vector of arrays to extend state vector of ode if required
    void operator() ( const std::array<double,3> &x , std::array<double,3> &dxdt , double t )
    {
        // // Boost integrator e.g. integrate_adaptive requires a variable t
        // // t is not used by the ode
        // // Create a dummy use case to avoid warnings
        t = 0.0;

        double pi = 2*acos(0.0);

        double rdelta = pm.delta * pi / 180; // Rudder angle in radians
   
        double u = x[0]; // Surge velocity at center of gravity
        double v = x[1]; // Lateral velocity at center of gravity
        double r = x[2]; // Yaw rate

        double U = sqrt(pow(u,2) + pow(v - r * pm.x_G,2)) + t; // Resultant speed (u^2 + v_m^2)

        double beta = 0.0; // Hull drift angle at midship
        double v_dash = 0.0; // non-dimensionalized lateral velocity defined by v/U
        double r_dash = 0.0; // non-dimensionalized yaw rate by r L_pp/U
        
        if (U != 0.0)
        {
            beta = asin(-(v - r * pm.x_G) / U);
            v_dash = v / U;
            r_dash = r * pm.L_pp / U;
        } 
        //w_P = w_P0
        double w_P = pm.w_P0 * std::exp(-4.0 * pow(beta - pm.x_P_dash * r_dash,2)); // Wake coefficient at propeller position in maneuvering motions

        double J = 0.0; // J_p Propeller advanced ratio
        if (pm.npm != 0.0)
        {
            J = (1 - w_P) * u / (pm.npm * pm.D_p);
        }
        
        double K_T = pm.k_0 + pm.k_1 * J + pm.k_2 * std::pow(J,2); // Propeller thrust open water characteristic
        
        double beta_R = beta - pm.l_r_dash * r_dash; // Effective inflow angle to rudder in maneuvering motions
        
        double Y_R = pm.Y_R_minus;  // Lateral force around midship by steering
        if (beta_R < 0.0)
        {
            Y_R = pm.Y_R_plus;
        }
        
        double v_R = U * Y_R * beta_R;  // Lateral inflow velocity components to rudder
        
        // 2 * acos(0.0) = PI
        // Longitudinal inflow velocity components to rudder
        double u_R = sqrt(pm.eta * (pm.kappa * pm.epsilon * 8.0 * pm.k_0 * std::pow(pm.npm,2) * std::pow(pm.D_p,4) / pow((2 * acos(0.0)), 2))); 
        if (J!=0)
        {
        u_R = u* (1 - w_P)* pm.epsilon* sqrt(pm.eta * pow(1.0 + pm.kappa * (sqrt(1.0 + 8.0 * K_T / (pi * pow(J, 2))) - 1),2)+ (1 - pm.eta));
        }  
                
        double U_R = sqrt(pow(u_R, 2) + pow(v_R, 2)); // Resultant inflow velocity to rudder
        
        double alpha_R = rdelta - atan2(v_R, u_R); // Effective inflow angle to rudder

        double F_N = 0.5 * pm.A_R * pm.roh * pm.f_alpha * pow(U_R, 2) * sin(alpha_R); // Froude number based on ship length
        
        // Surge force around midship acting on ship hull except added mass components X P
        double X_H = ( 
            0.5
            * pm.roh
            * pm.L_pp
            * pm.d
            * pow(U ,2)
            * (
                -pm.R_0_dash
                + pm.X_vv_dash * pow(v_dash ,2)
                + pm.X_vr_dash * v_dash * r_dash
                + pm.X_rr_dash * pow(r_dash, 2)
                + pm.X_vvvv_dash * pow(v_dash, 4)
            )
        );

        double X_R = -(1 - pm.t_R) * F_N * sin(rdelta); // Surge force around midship by steering
        X_R = -(1 - pm.t_R) * F_N * sin(rdelta);
        
        double X_P = (1 - pm.t_P) * pm.roh * K_T * pow(pm.npm, 2) * pow(pm.D_p, 4);
        
        // Lateral force around midship acting on ship hull
        double Y_H = (
            0.5
            * pm.roh
            * pm.L_pp
            * pm.d
            * pow(U, 2)
            * (
                pm.Y_v_dash * v_dash
                + pm.Y_r_dash * r_dash
                + pm.Y_vvv_dash * pow(v_dash, 3)
                + pm.Y_vvr_dash * pow(v_dash, 2) * r_dash
                + pm.Y_vrr_dash * v_dash * pow(r_dash, 2)
                + pm.Y_rrr_dash * pow(r_dash, 3)
            )
        );
        Y_R = -(1 + pm.a_H) * F_N * cos(rdelta); // Lateral force around midship by steering
        
        // Yaw moment around midship acting on ship hull
        double N_H = (
            0.5
            * pm.roh
            * pow(pm.L_pp, 2)
            * pm.d
            * pow(U, 2)
            * (
                pm.N_v_dash * v_dash
                + pm.N_r_dash * r_dash
                + pm.N_vvv_dash * pow(v_dash, 3)
                + pm.N_vvr_dash * pow(v_dash, 2) * r_dash
                + pm.N_vrr_dash * v_dash * pow(r_dash, 2)
                + pm.N_rrr_dash * pow(r_dash, 3)
            )
        );
        double N_R = -(pm.x_R + pm.a_H * pm.x_H) * F_N * cos(rdelta); // Yaw moment around midship by steering
    
        // Surge acceleration at center of gravity
        dxdt[0] = ((X_H + X_R + X_P) + (pm.m + pm.m_y) * v * r + pm.x_G * pm.m * pow(r, 2)) / (pm.m + pm.m_x);
        
        // Lateral acceleration at center of gravity
        dxdt[1] = (
            pow(pm.x_G, 2) * pow(pm.m, 2) * u * r
            - (N_H + N_R) * pm.x_G * pm.m
            + ((Y_H + Y_R) - (pm.m + pm.m_x) * u * r) * (pm.I_zG + pm.J_z + pow(pm.x_G, 2) * pm.m)
        ) / ((pm.I_zG + pm.J_z + pow(pm.x_G, 2) * pm.m) * (pm.m + pm.m_y) - pow(pm.x_G, 2) * pow(pm.m, 2));
        
        // Yaw acceleration
        dxdt[2] = (N_H + N_R - pm.x_G * pm.m * (dxdt[1] + u * r)) / (pm.I_zG + pm.J_z + pow(pm.x_G, 2) * pm.m);
    }
};
#endif //MMG_ODE_HPP