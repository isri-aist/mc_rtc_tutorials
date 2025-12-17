set(controller_SRC
  DualArmController.cpp
)

set(controller_HDR
  DualArmController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DDualArmController_EXPORTS")

