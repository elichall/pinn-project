# 1. Create a variable called SOURCES that holds a list of your .cpp files.
# Whenever you add a new controller, you just add its filename to this list.
set(SOURCES
    main.cpp
    plant/ManipulatorPlant.cpp
    trajectory/TrajectoryGenerator.cpp
    models/RobotModel.cpp
    ipc/SharedMemoryLink.cpp
    sensors/RobotSensors.cpp
)

# 2. Create the actual executable program and name it "RoboticSim".
# We pass it the ${SOURCES} variable so it knows which files to compile.
add_executable(RoboticSim ${SOURCES})

# 3. Tell the compiler where your header files (.h) live.
# "PRIVATE" means these headers are only used by RoboticSim, not shared with other targets.
target_include_directories(RoboticSim PRIVATE include plant trajectory controllers models ipc sensors graphics)

# 4. Link the necessary libraries to your executable.
target_link_libraries(RoboticSim
    PRIVATE
    Eigen3::Eigen  # Eigen headers (found via find_package or vendor fallback)
    Boost::boost   # Boost headers for Boost.Interprocess
    pthread        # POSIX Threading: Required by Boost under the hood
    rt             # POSIX Real-Time: Required for Linux shared memory (shm_open)
)

# 5. Create the separate graphics application executable
add_executable(GraphicsApp
    graphics/GraphicsApp.cpp
    ipc/SharedMemoryLink.cpp
)
target_include_directories(GraphicsApp PRIVATE
    include
    ipc
    graphics
)
target_link_libraries(GraphicsApp PRIVATE InstanceSimEngine::GraphicsEngine pthread rt)
