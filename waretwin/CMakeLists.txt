cmake_minimum_required(VERSION 3.8)
project(waretwin_web)
find_package(ament_cmake REQUIRED)
find_program(NPM_EXECUTABLE npm REQUIRED)
find_program(NODE_EXECUTABLE node REQUIRED)
execute_process(COMMAND ${NODE_EXECUTABLE} -e "if(Number(process.versions.node.split('.')[0])<22)process.exit(1)"
  RESULT_VARIABLE node_status)
if(NOT node_status EQUAL 0)
  message(FATAL_ERROR "Node.js 22 or newer is required to build the frontend")
endif()
# Build in the build tree: neither install nor runtime needs frontend sources.
file(GLOB_RECURSE FRONTEND_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/frontend/src/*
  ${CMAKE_CURRENT_SOURCE_DIR}/frontend/public/*)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/frontend/dist/index.html
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/frontend/src ${CMAKE_CURRENT_BINARY_DIR}/frontend/src
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/frontend/public ${CMAKE_CURRENT_BINARY_DIR}/frontend/public
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/frontend/package.json ${CMAKE_CURRENT_SOURCE_DIR}/frontend/package-lock.json ${CMAKE_CURRENT_SOURCE_DIR}/frontend/tsconfig.json ${CMAKE_CURRENT_SOURCE_DIR}/frontend/vite.config.ts ${CMAKE_CURRENT_SOURCE_DIR}/frontend/index.html ${CMAKE_CURRENT_BINARY_DIR}/frontend/
  COMMAND ${NPM_EXECUTABLE} ci --no-audit --no-fund
  COMMAND ${CMAKE_COMMAND} -E env VITE_API_BASE_URL= VITE_API_URL= VITE_WS_BASE_URL= VITE_WS_URL= VITE_BACKEND_PORT= VITE_DEMO_COMPACT_VIEW=true ${NPM_EXECUTABLE} run build
  DEPENDS ${FRONTEND_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/package.json
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/package-lock.json
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/tsconfig.json
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/vite.config.ts
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/index.html
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/frontend
  COMMENT "Building production WareTwin frontend" VERBATIM)
add_custom_target(frontend ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/frontend/dist/index.html)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/frontend)
install(DIRECTORY launch runtime DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY backend/config backend/twin DESTINATION share/${PROJECT_NAME}/backend
  PATTERN "__pycache__" EXCLUDE PATTERN "tests" EXCLUDE)
install(FILES backend/manage.py backend/requirements.txt DESTINATION share/${PROJECT_NAME}/backend)
install(FILES ../tools/export_gazebo_world.py DESTINATION share/${PROJECT_NAME}/backend/tools)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/frontend/dist/ DESTINATION share/${PROJECT_NAME}/frontend)
ament_package()
