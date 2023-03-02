#include <igl/embree/EmbreeIntersector.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/read_triangle_mesh.h>

int main(int argc, char *argv[]) {
  // Load a mesh from file
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(argc > 1 ? argv[1] : "../screwdriver.off", V, F);

  // Set up viewer
  igl::opengl::glfw::Viewer vr;
  vr.data().set_mesh(V, F);

  // Add two points to the viewer
  Eigen::MatrixXd P(2, 3);

  // Define red and green color
  Eigen::RowVector3d red(1, 0, 0);
  Eigen::RowVector3d green(0, 1, 0);

  // Make a grid in the YZ plane
  int rows = 3;
  int cols = 4;
  double spacing = 0.01;
  // Eigen::RowVector3d grid_center = Eigen::RowVector3d(0, rows * spacing / 2.0, cols * spacing / 2.0);
  Eigen::MatrixXd gridPoints = Eigen::MatrixXd::Zero(rows * cols, 3);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      Eigen::RowVector3d point(0, i * spacing, j * spacing);
      int index = i * cols + j;
      gridPoints.row(index) = point;  // - grid_center;
    }
  }

  double distance_between_points = 0.05;
  double d2 = distance_between_points / 2.0;
  Eigen::MatrixXd sensorStarts = gridPoints.rowwise() + Eigen::RowVector3d(-d2, 0, 0);
  Eigen::MatrixXd sensorEnds = gridPoints.rowwise() + Eigen::RowVector3d(d2, 0, 0);

  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  vr.plugins.push_back(&imgui_plugin);

  // Add a 3D gizmo plugin
  igl::opengl::glfw::imgui::ImGuizmoWidget gizmo;
  gizmo.operation = ImGuizmo::TRANSLATE;
  imgui_plugin.widgets.push_back(&gizmo);

  // Initialize ImGuizmo at mesh centroid
  gizmo.T.block(0, 3, 3, 1) = 0.5 * (V.colwise().maxCoeff() + V.colwise().minCoeff()).transpose().cast<float>();

  // Update can be applied relative to this remembered initial transform
  const Eigen::Matrix4f T0 = gizmo.T;

  // Attach callback to apply imguizmo's transform to mesh
  gizmo.callback = [&](const Eigen::Matrix4f &T) {
    const Eigen::Matrix4d TT = (T * T0.inverse()).cast<double>().transpose();
    // Transformed vertices
    Eigen::MatrixXd V_transformed = (V.rowwise().homogeneous() * TT).rowwise().hnormalized();
    vr.data().set_vertices(V_transformed);
    vr.data().compute_normals();

    vr.data().clear_points();
    vr.data().clear_edges();

    for (int i = 0; i < sensorStarts.rows(); i++) {
      Eigen::RowVector3d start = sensorStarts.row(i);
      Eigen::RowVector3d end = sensorEnds.row(i);
      Eigen::RowVector3d dir = end - start;
      igl::Hit hit;
      igl::embree::EmbreeIntersector ei;
      ei.init(V_transformed.cast<float>(), F.cast<int>());
      bool hit_success = ei.intersectRay(start.cast<float>(), dir.cast<float>(), hit);
      std::cout << "embree hit success: " << hit_success << std::endl;
      // print hit info
      std::cout << "hit id: " << hit.id << std::endl;
      std::cout << "hit t: " << hit.t << std::endl;

      Eigen::RowVector3d color = hit_success && hit.t <= 1.0 ? red : green;
      vr.data().add_points(start, color);
      vr.data().add_points(end, color);
      vr.data().add_edges(start, end, color);
    }
  };
  // Blender-style keyboard shortcuts for operation
  vr.callback_key_pressed = [&](decltype(vr) &, unsigned int key, int mod) {
    switch (key) {
      case ' ':
        gizmo.visible = !gizmo.visible;
        return true;
      case 'G':
      case 'g':
        gizmo.operation = ImGuizmo::TRANSLATE;
        return true;
      case 'R':
      case 'r':
        gizmo.operation = ImGuizmo::ROTATE;
        return true;
      case 'S':
      case 's':
        gizmo.operation = ImGuizmo::SCALE;
        return true;
    }
    return false;
  };

  igl::opengl::glfw::imgui::ImGuiMenu menu;
  imgui_plugin.widgets.push_back(&menu);

  std::cout << R"(
G,g  Switch to translate operation
R,r  Switch to rotate operation
S,s  Switch to scale operation
)";
  vr.launch();
}
