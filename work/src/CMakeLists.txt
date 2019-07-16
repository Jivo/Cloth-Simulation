
# Source Files
#
# If you add any new files, remember to add them here
SET(sources
  cloth.cpp
  cloth.hpp
  constraint.hpp
  distance_constraint.cpp
  distance_constraint.hpp
  bending_constraint.cpp
  bending_constraint.hpp
  collision_constraint.cpp
  collision_constraint.hpp
  grid_maker.hpp
  grid_maker_interface.cpp
  grid_maker_interp.cpp
  sphere.cpp
  sphere.hpp
  entity.cpp
  entity.hpp
  particle.cpp
  particle.hpp
  pbd.cpp
  pbd.hpp
  scene.cpp
  scene.hpp
  opengl.hpp
  main.cpp
)

add_executable(${CGRA_PROJECT} ${sources})

add_subdirectory(cgra)

target_source_group_tree(${CGRA_PROJECT})

target_link_libraries(${CGRA_PROJECT} PRIVATE ${OPENGL_LIBRARY})
target_link_libraries(${CGRA_PROJECT} PRIVATE glfw ${GLFW_LIBRARIES})
target_link_libraries(${CGRA_PROJECT} PRIVATE glew glm imgui)

target_include_directories(${CGRA_PROJECT} PRIVATE "${PROJECT_SOURCE_DIR}/src")

# Set the source directory as a preprocessor define, used to make sure that the relative paths
# work correctly, regardless of where the project is run fron (as long as it's run on the same
# machine it was built on).
target_compile_definitions(${CGRA_PROJECT} PRIVATE "-DCGRA_SRCDIR=\"${PROJECT_SOURCE_DIR}\"")