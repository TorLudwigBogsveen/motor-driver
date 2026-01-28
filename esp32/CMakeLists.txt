idf_component_register(SRCS 
"../core/error.cpp" 
"../core/input.cpp" 
"../core/controller.cpp" 
"can/can_driver.cpp"
"log.cpp"
"main.cpp"
"clock.cpp"
                    
INCLUDE_DIRS "../include"

REQUIRES driver
)