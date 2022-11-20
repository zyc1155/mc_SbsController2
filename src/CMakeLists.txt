set(controller_SRC
  SbsControllerZyc.cpp
  driver.cpp
  test.cpp
)

set(controller_HDR
  SbsControllerZyc.h
  driver.h
  mem.h
  test.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsControllerZyc_EXPORTS")
find_library(nifalcon usb-1.0)
target_link_libraries(${PROJECT_NAME} PUBLIC nifalcon usb-1.0)

