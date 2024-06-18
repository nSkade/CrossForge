// std
#include <vector>
#include <string>
#include <memory>
#include <map>

// lib

// eigen
#ifndef NDEBUG
#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#endif
#include <Eigen/Eigen>

#define JSON_DIAGNOSTICS 1
#include <nlohmann/json.hpp>

// igl
#include <igl/unproject_ray.h>
#include <igl/ray_box_intersect.h>
#include <igl/ray_sphere_intersect.h>
#include <igl/ray_mesh_intersect.h>

// imgui
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>
#include <ImSequencer.h>
#include <ImCurveEdit.h>
