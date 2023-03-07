Jonas Mahler 12.2022

# Description
This .cpp program is a 3 Degree of Freedom model of     
a ship after Maneuvering Modeling Group (MMG) model.    
It is proposed by the Japan Society of Naval Architects and
Ocean Engineers and shall serve as a prototype of maneuvering
prediction method for ships     

For more information look into 

- H. Yasukawa • Y. Yoshimura 2014 "Introduction of MMG standard     
method for ship maneuvering predictions" in JASNAOE 

The program my also serve as a template for ship     
models with a higher degree of freedom, e.g. 6DoF.      

Odeint is used for integration, which is     
included in boost. The differential equation     
as well as the observer to the function has to     
be passed to the odeint integrator function.        

Odeint does not allow member functions to be     
passed to the integrator function. Thus the ode     
and the observer are functors.They can be found         
in mmg_ode.hpp and mmg_obs.hpp respectively.     

# TODO
Test the code with Figure 14 of H. Yasukawa 2014

# Build the example code

Cmake is used to create the build files.  

By building this code, catch2 is downloaded     
and installed in the build folder by cmake.    
Therefore the first build may take some time.          

This will also build an executable with the tests     
that can be run (see more in Test the code).         

Clone the code
```
git clone git@github.com:GitRepJo/mmg.git
```
Create a build directory in the new git directory
```
cd /home/$USER/mmg 
mkdir -p build 
```
Create the build files with cmake and build the code
```
cmake -DCMAKE_BUILD_TYPE=Releases -S . -B build && cmake --build build
```

# Run the example code 
An example simulation run with the program can be     
found in main.cpp.        

Parameters for the simulation and the differential     
equation can be read from a .yaml file or specified     
directly in the main function. This is shown in main.cpp     
that may serve as an example to run the code.       

Run the code
```
cd /home/$USER/mmg/build
./mmg
```

This is a terminal sample of the program running     
as specified in main.cpp and config.yaml    
```
...    

t[sec]: 9.67 x[m]: 19.37 y[m]: 29.99 yaw[deg]: 189.79 x_vel[m/s]: 2.16 y_vel[m/s]: -0.64

t[sec]: 9.74 x[m]: 19.21 y[m]: 30.01 yaw[deg]: 190.51 x_vel[m/s]: 2.15 y_vel[m/s]: -0.64

t[sec]: 9.81 x[m]: 19.05 y[m]: 30.03 yaw[deg]: 191.25 x_vel[m/s]: 2.13 y_vel[m/s]: -0.64

t[sec]: 9.88 x[m]: 18.89 y[m]: 30.04 yaw[deg]: 191.99 x_vel[m/s]: 2.11 y_vel[m/s]: -0.63

t[sec]: 9.96 x[m]: 18.73 y[m]: 30.05 yaw[deg]: 192.72 x_vel[m/s]: 2.1 y_vel[m/s]: -0.63

t[sec]: 10 x[m]: 18.64 y[m]: 30.06 yaw[deg]: 193.14 x_vel[m/s]: 2.09 y_vel[m/s]: -0.62

```
# VS Code

If the Development Environment VS Code is used, build and test tasks can be called directly from within VSCode
Open the package in VS Code

```
cd /home/$USER/mmg

code .
```
With Shift+Control+B the package can be build.

With Shift+Control+P -> Task: Run Test Task -> test tests can be run.