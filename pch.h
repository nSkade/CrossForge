#include <Eigen/Eigen>
#include <vector>
#include <string>

#define JSON_DIAGNOSTICS 1
#include <nlohmann/json.hpp>

#include <map>

#include <igl/unproject_ray.h>
#include <igl/ray_box_intersect.h>
#include <igl/ray_mesh_intersect.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>
#include <ImSequencer.h>
#include <ImCurveEdit.h>

