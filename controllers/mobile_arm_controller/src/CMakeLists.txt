set(controller_SRC
  MobileArmController.cpp
)

set(controller_HDR
  MobileArmController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMobileArmController_EXPORTS")

