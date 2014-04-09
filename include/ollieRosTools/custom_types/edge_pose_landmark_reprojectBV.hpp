
#ifndef EDGE_POSE_LANDMARKBV
#define EDGE_POSE_LANDMARKBV

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/hyper_graph_action.h>

#include <ollieRosTools/custom_types/vertex_pose.hpp>
#include <ollieRosTools/custom_types/vertex_landmarkxyz.hpp>


class EdgePoseLandmarkReprojectBV : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexLandmarkXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePoseLandmarkReprojectBV() {
        _vertices.resize(2);
        resize(2);
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void         computeError();

};

/**
 * \brief visualize the 3D pose vertex
 */
class EdgePoseLandmarkReprojectBVDrawAction: public g2o::DrawAction {
public:
    EdgePoseLandmarkReprojectBVDrawAction();
    virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, g2o::HyperGraphElementAction::Parameters* params_);
    g2o::HyperGraphElementAction* _cacheDrawActions;
protected:
    virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
};


#endif // EDGE_POSE_POSE
