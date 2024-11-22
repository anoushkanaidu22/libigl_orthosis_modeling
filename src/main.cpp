#include <igl/opengl/glfw/Viewer.h>
#include <igl/cylinder.h>
#include <igl/cat.h>
#include <Eigen/Geometry>

Eigen::MatrixXd V_orig; //og vertices
Eigen::MatrixXd V; //curr vertices
Eigen::MatrixXi F;
float joint1_angle = 0.0f;
float joint2_angle = 0.0f;

void update_finger_position(igl::opengl::glfw::Viewer& viewer) {
   //reset to original position
   V = V_orig;
   
   //joint positions
   Eigen::Vector3d joint1 = V_orig.row(V_orig.rows()/3); //first joint at end of first cylinder
   Eigen::Vector3d joint2 = V_orig.row(2*V_orig.rows()/3); //second joint at end of second cylinder
   
   //rotate around joints
   for(int i = V_orig.rows()/3; i < V_orig.rows(); i++) {
       Eigen::Vector3d v = V.row(i);
       Eigen::AngleAxisd rotation1(joint1_angle, Eigen::Vector3d::UnitX());
       v = rotation1 * (v - joint1) + joint1;
       V.row(i) = v;
       
       if(i >= 2*V_orig.rows()/3) {
           Eigen::AngleAxisd rotation2(joint2_angle, Eigen::Vector3d::UnitX());
           v = rotation2 * (v - joint2) + joint2;
           V.row(i) = v;
       }
   }
   
   viewer.data().set_vertices(V);
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
   switch(key) {
       case '1': joint1_angle += 0.1; break;
       case '2': joint1_angle -= 0.1; break;
       case '3': joint2_angle += 0.1; break;
       case '4': joint2_angle -= 0.1; break;
       case 'R': //reset
           joint1_angle = 0;
           joint2_angle = 0;
           break;
   }
   update_finger_position(viewer);
   return false;
}

int main(int argc, char *argv[]) {
   //cylinder creations
   Eigen::MatrixXd V1, V2, V3;
   Eigen::MatrixXi F1, F2, F3;
   
   igl::cylinder(20, 2, V1, F1);
   V2 = V1; F2 = F1;
   V3 = V1; F3 = F1;
   
   //scale +  position cylinders
   V1 *= Eigen::Scaling(0.1, 0.1, 0.3);
   V2 *= Eigen::Scaling(0.09, 0.09, 0.25);
   V3 *= Eigen::Scaling(0.08, 0.08, 0.2);
   
   V2.col(2).array() += 0.3;
   V3.col(2).array() += 0.55;
   
   igl::cat(1, V1, V2, V_orig);
   V_orig = igl::cat(1, V_orig, V3);
   F2.array() += V1.rows();
   F3.array() += V1.rows() + V2.rows();
   igl::cat(1, F1, F2, F);
   F = igl::cat(1, F, F3);
   
   V = V_orig; 
   
   igl::opengl::glfw::Viewer viewer;
   viewer.data().set_mesh(V, F);
   viewer.callback_key_down = &key_down;
   viewer.data().set_face_based(true);
   viewer.core().align_camera_center(V);
   viewer.launch();
}

