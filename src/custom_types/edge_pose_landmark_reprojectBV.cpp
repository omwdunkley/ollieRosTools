#include <ollieRosTools/custom_types/edge_pose_landmark_reprojectBV.hpp>
#include <ollieRosTools/custom_types/draw_functions.hpp>

bool EdgePoseLandmarkReprojectBV::write(std::ostream& os) const {
    os  << measurement()[0] << " "
        << measurement()[1] << " ";

    //information matrix
    for (int i=0; i<information().rows(); i++)
        for (int j=0; j<information().cols(); j++) {
            os <<  information()(i,j) << " ";
        }

    return true;
}

bool EdgePoseLandmarkReprojectBV::read(std::istream& is) {
    double x,y;
    is >> x;
    is >> y;

    setMeasurement(Eigen::Vector2d(x,y));

    if (is.bad()) {
        return false;
    }

    for (int i=0; i<information().rows() && is.good(); i++)
        for (int j=0; j<information().cols() && is.good(); j++) {
            is >> information()(i,j);
        }

    // will never gethere
    if (is.bad()) {
        //  we overwrite the information matrix with the Identity
        information().setIdentity();
    }

    return true;
}

void EdgePoseLandmarkReprojectBV::computeError() {

    const VertexPose* w_T_cam = static_cast<const VertexPose*>(_vertices[0]);
    const VertexLandmarkXYZ* P_world = static_cast<const VertexLandmarkXYZ*>(_vertices[1]);

    // no camera model, as just using bearing vectors
    // measurement is x,y of bearing vector

    // bearing vector from cam to point in world
    Eigen::Vector3d cam_T_lm = (w_T_cam->estimate().inverse() * P_world->estimate());
    Eigen::Vector2d px(cam_T_lm[0]/cam_T_lm[2], cam_T_lm[1]/cam_T_lm[2]);
    //_error = 1.0 - (_measurement.transpose() * cam_T_lm);
    _error =  _measurement - px;

}

EdgePoseLandmarkReprojectBVDrawAction::EdgePoseLandmarkReprojectBVDrawAction(): g2o::DrawAction(typeid(EdgePoseLandmarkReprojectBV).name()) {

}

bool EdgePoseLandmarkReprojectBVDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
    if (!DrawAction::refreshPropertyPtrs(params_)) {
        return false;
    }
    return true;
}

g2o::HyperGraphElementAction* EdgePoseLandmarkReprojectBVDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_) {

    if (typeid(*element).name()!=_typeName) {
        return 0;
    }

    refreshPropertyPtrs(params_);

    if (! _previousParams) {
        return this;
    }

    if (_show && !_show->value()) {
        return this;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Get data
    const EdgePoseLandmarkReprojectBV* reproj_edge_ptr = static_cast<EdgePoseLandmarkReprojectBV*>(element);
    const VertexPose* w_T_cam = static_cast<const VertexPose*>(reproj_edge_ptr->vertices()[0]);
    const VertexLandmarkXYZ* P_world  = static_cast<const VertexLandmarkXYZ*>(reproj_edge_ptr->vertices()[1]);

    Eigen::Vector3d camera    = w_T_cam->estimate().translation();
    Eigen::Vector3d landmark  = P_world->estimate();
    Eigen::Affine3d world2cam = w_T_cam->estimate();

    // get bearing vector in world coords
    Eigen::Vector2d px = reproj_edge_ptr->measurement();
    Eigen::Vector3d bv        = world2cam* Eigen::Vector3d(px[0], px[1], 1.0);


    //get bearing vector cam->point in cam frame, then put on image plane
    Eigen::Vector3d bvP = world2cam.inverse()*landmark; // vector cam -> point cam frame
    bvP = Eigen::Vector3d(bvP[0]/bvP[2], bvP[1]/bvP[2], 1.0);
    Eigen::Vector3d pxP = world2cam * bvP;




    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Draw points

    // gl setup
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);

    // draw 3D will draw rays for omni pixels, and 3D points for stereo
    glLineWidth(0.1);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(camera[0], camera[1], camera[2]);
    glVertex3f(landmark[0], landmark[1], landmark[2]);
    glEnd();

    // draw bearing vector to image plane
    glLineWidth(2);
    glColor3f(0.0f,1.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(camera[0], camera[1], camera[2]);
    glVertex3f(bv[0], bv[1], bv[2]);
    glEnd();

    // draw error on image plane
    glLineWidth(1);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(pxP[0], pxP[1], pxP[2]);
    glVertex3f(bv[0], bv[1], bv[2]);
    glEnd();


    // cleanup opengl settings
    glEnable(GL_LIGHTING);
    glPopAttrib();
    return this;
}
