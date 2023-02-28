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

  V.array().col(2) += 0.025;

  // Make new mesh with triangle that we know intersects the ray
  // Eigen::MatrixXd V(3, 3);
  // Eigen::MatrixXi F(1, 3);
  // V << 0, -0.01, -0.01, 0, 0, 0.01, 0, 0.01, 0;
  // F << 0, 1, 2;

  // Set up viewer
  igl::opengl::glfw::Viewer vr;
  vr.data().set_mesh(V, F);

  // Add two points to the viewer
  Eigen::MatrixXd P(2, 3);
  double distance_between_points = 0.05;
  double d2 = distance_between_points / 2.0;
  P << -d2, 0, 0, d2, 0, 0;
  vr.data().add_points(P, Eigen::RowVector3d(1, 0, 0));
  vr.data().add_edges(P.row(0), P.row(1), Eigen::RowVector3d(1, 0, 0));

  igl::Hit hit;
  Eigen::RowVector3d start = P.row(0);
  Eigen::RowVector3d dir = P.row(1) - P.row(0);
  std::cout << "start: " << start << std::endl;
  std::cout << "dir: " << dir << std::endl;
  std::cout << "V.rows(): " << V.rows() << std::endl;
  std::cout << "F.rows(): " << F.rows() << std::endl;

  // Warning: this checks all triangles
  bool hit_success = igl::ray_mesh_intersect(start, dir, V, F, hit);
  std::cout << "hit success: " << hit_success << std::endl;
  std::cout << "hit.t: " << hit.t << std::endl;

  igl::Hit hit2;
  igl::embree::EmbreeIntersector ei;
  ei.init(V.cast<float>(), F.cast<int>());
  bool hit_success2 = ei.intersectRay(start.cast<float>(), dir.cast<float>(), hit2);
  std::cout << "embree hit success: " << hit_success2 << std::endl;

  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  vr.plugins.push_back(&imgui_plugin);

  // Add a 3D gizmo plugin
  igl::opengl::glfw::imgui::ImGuizmoWidget gizmo;
  imgui_plugin.widgets.push_back(&gizmo);
  // Initialize ImGuizmo at mesh centroid
  gizmo.T.block(0, 3, 3, 1) =
      0.5 * (V.colwise().maxCoeff() + V.colwise().minCoeff())
                .transpose()
                .cast<float>();
  // Update can be applied relative to this remembered initial transform
  const Eigen::Matrix4f T0 = gizmo.T;
  // Attach callback to apply imguizmo's transform to mesh
  gizmo.callback = [&](const Eigen::Matrix4f &T) {
    const Eigen::Matrix4d TT = (T * T0.inverse()).cast<double>().transpose();
    vr.data().set_vertices(
        (V.rowwise().homogeneous() * TT).rowwise().hnormalized());
    vr.data().compute_normals();
  };
  // Maya-style keyboard shortcuts for operation
  vr.callback_key_pressed = [&](decltype(vr) &, unsigned int key, int mod) {
    switch (key) {
    case ' ':
      gizmo.visible = !gizmo.visible;
      return true;
    case 'W':
    case 'w':
      gizmo.operation = ImGuizmo::TRANSLATE;
      return true;
    case 'E':
    case 'e':
      gizmo.operation = ImGuizmo::ROTATE;
      return true;
    case 'R':
    case 'r':
      gizmo.operation = ImGuizmo::SCALE;
      return true;
    }
    return false;
  };

  igl::opengl::glfw::imgui::ImGuiMenu menu;
  imgui_plugin.widgets.push_back(&menu);

  std::cout << R"(
W,w  Switch to translate operation
E,e  Switch to rotate operation
R,r  Switch to scale operation
)";
  vr.launch();
}
