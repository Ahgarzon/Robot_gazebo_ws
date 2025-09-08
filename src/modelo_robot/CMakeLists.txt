cmake_minimum_required(VERSION 3.10)
project(modelo_robot)

find_package(ament_cmake REQUIRED)

# Copiamos TODO lo que el paquete necesita en install/share/modelo_robot
install(
  DIRECTORY
    launch
    config          # ← nav2_params.yaml, mapper_params_online_async.yaml…
    maps
    meshes
    rviz
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

