1. Open a terminal
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/library_path
   Run matlab from the command line

2. Set model parameters for ROS code generation
   Set “Simulation target/Include directories” to “./”
   Set “Simulation target/Libraries” to “libCPLEXsolver_cwrapper.so”
   In “Code generation/Custom code” flag “Use the custom code settings as Simulation target”

To switch between different gcc versions:
sudo update-alternatives --config gcc
