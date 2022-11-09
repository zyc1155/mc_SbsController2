set(controller_SRC
  SbsControllerZyc.cpp
)

set(controller_HDR
  SbsControllerZyc.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsControllerZyc_EXPORTS")

