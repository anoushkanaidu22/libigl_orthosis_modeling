#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/bbw.h>
#include <igl/massmatrix.h>
#include "rigging.h"
#include "physics.h"

int main(int argc, char *argv[])
{
    //nesh data
    Eigen::MatrixXd V;  //vertices
    Eigen::MatrixXi F;  //faces
    
    //read .obj file to load mesh
    igl::readOBJ("../models/hand.obj", V, F);
    
    Rigging rigging;
    rigging.initialize(V, F);
    
    Physics physics;
    physics.initialize(V, F, rigging.getWeights());
    
    //setup viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    
    //animation callback
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & viewer)->bool
    {
        physics.update();
        viewer.data().set_vertices(physics.getVertices());
        return false;
    };
    
    viewer.launch();
    return 0;
}
