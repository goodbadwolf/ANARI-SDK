## Copyright 2022 The Khronos Group
## SPDX-License-Identifier: Apache-2.0
include(FetchContent)
project(local_vtkm LANGUAGES CXX)

option(VTKm_ENABLE_TBB     "" ON)
option(VTKm_ENABLE_TESTING "" OFF)

anari_sdk_fetch_project(
  NAME anari_khilari_vtkm
  URL "https://gitlab.kitware.com/vtk/vtk-m/-/archive/v2.0.0/vtk-m-v2.0.0.zip"
  ADD_SUBDIR
)

project_add_library(INTERFACE)
project_link_libraries(INTERFACE vtkm_cont vtkm_io vtkm_rendering vtkm_source)
project_include_directories(INTERFACE ${anari_khilari_vtkm_LOCATION}/include)
