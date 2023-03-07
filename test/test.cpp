#include <catch2/catch_test_macros.hpp>
#include <mmg_sim.hpp>

TEST_CASE( "Test the Nomoto model")
{
    MMGSim exampleSim;
    MMGSim::parameterSim pS;
    parameterMMG pM;
    MMGSim::resultMMG resM;

    // Read in values from file
    pM = exampleSim.readMMG("./test_mmg_config.yaml");
    pS = exampleSim.readSimulation("./test_sim_config.yaml");
    
    SECTION("Read values should match expectation")
    {   
        pS.terminal_output = false;
        // Compare only a selection of values
        REQUIRE( pM.A_R == 0.0539);
        REQUIRE( pS.initYVelocity == 7.9738888889) ;
    }
    
    // Use Figure 3 of H. Yasukawa • Y. Yoshimura 2014 "Introduction of MMG standard method for ship maneuvering predictions" in JASNAOE 
    // Deviations can occur because from the publication the used rps of the propeller are not described.
    SECTION("Compare model with figure 13 x0/L= 2.0 & y0/L = 3.2 with L=7m")
    {   
        pS.terminal_output = false;
        pM.delta = 35.0;
        pM.npm = 70.0;
        pS.time = 11.5;
        pS.step = 0.1;
        
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.yaw_rate.size();
        double end_x_pos = resM.x_pos.at(s_result-1);
        double end_y_pos = resM.y_pos.at(s_result-1);
        
        REQUIRE( (end_x_pos/pM.L_pp > 1.9 ) & (end_x_pos/pM.L_pp < 2.1) );
        REQUIRE( (end_y_pos/pM.L_pp > 3.0 ) & (end_y_pos/pM.L_pp < 3.3) );
    }
    
    // Use Figure 3 of H. Yasukawa • Y. Yoshimura 2014 "Introduction of MMG standard method for ship maneuvering predictions" in JASNAOE 
    // Deviations can occur because from the publication the used rps of the propeller are not described.
     SECTION("Compare model with figure 13 x0/L= 2.0 & y0/L = 0.9 with L=7m")
    {   
     
        pM.delta = 35.0;
        pM.npm = 70.0;
        pS.time = 20.42;
        pS.step = 0.1;
        pS.terminal_output = false;
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.yaw_rate.size();
         double end_x_pos = resM.x_pos.at(s_result-1);
        double end_y_pos = resM.y_pos.at(s_result-1);
        
        REQUIRE( (end_x_pos/pM.L_pp > 1.3 ) & (end_x_pos/pM.L_pp < 2.0) );
        REQUIRE( (end_y_pos/pM.L_pp > 0.8 ) & (end_y_pos/pM.L_pp < 1.1) );
    }
    

    SECTION("position after 1 second and rudder angle 0 with constant velocity")
    {   
        pM.delta = 0.0;

        pS.time = 1;
        pS.step = 0.1;

        pS.initYVelocity = 0.0;
        pS.initXVelocity = 0.0;
      
        pS.terminal_output = false;
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.x_pos.size();

        // Round because of floating point errors
        double end_pos_x = round( resM.x_pos.at(s_result-1) * 100 ) / 100; 
        double end_pos_y = round( resM.y_pos.at(s_result-1) * 100 ) / 100; 
        
        REQUIRE(end_pos_x == 0.4);
        REQUIRE(end_pos_y == 0.0);
    }

       SECTION("position after 1 second and rudder angle 0 with constant velocity and initial heading to 90 degree ")
    {   
 
        pM.delta = 0.0;

        pS.time = 1.0;
        pS.step = 0.1;

        pS.initYVelocity = 0.0;
        pS.initXVelocity = 0.0;

        pS.initYaw = 1.570796;
        pS.terminal_output = false;
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.x_pos.size();

        // Round because of floating point errors
        double end_pos_x = round( resM.x_pos.at(s_result-1) * 100 ) / 100; 
        double end_pos_y = round( resM.y_pos.at(s_result-1) * 100 ) / 100; 
        
        REQUIRE(end_pos_x == 0.0);
        REQUIRE(end_pos_y == 0.4);
    }

    SECTION("yaw angle has to be in positive x,y section for positive rudder deflection")
    {   
        pM.delta = 30.0;
        pS.time = 1.0;
        pS.step = 0.1;
        pS.initYaw = 0.0;
        pS.initYVelocity = 0.0;
        pS.initXVelocity = 0.0;
        pS.terminal_output = true;
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.x_pos.size();

        // Round because of floating point errors
        double yaw = round( resM.yaw.at(s_result-1) * 100 ) / 100; 
    
        REQUIRE((yaw > 0) & (yaw < 90 ));
    }

    SECTION("yaw angle has to be in negative x,y section for negative rudder deflection")
    {   
        pM.delta = -30.0;
        pS.time = 1.0;
        pS.step = 0.1;
      
        pS.initYaw = 0.0;
        pS.terminal_output = false;
        
        resM = exampleSim.runSim(pM,pS);
        
        int s_result = resM.x_pos.size();

        // Round because of floating point errors
        double yaw = round( resM.yaw.at(s_result-1) * 100 ) / 100; 
    
        REQUIRE((yaw < 0) & (yaw > -90 ));
    }


}

