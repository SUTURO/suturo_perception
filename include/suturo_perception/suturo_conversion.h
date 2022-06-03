/**
 * Some conversation functions
 * Similiar to the rs::conversion paradigm
 * @author: Fenja Kollasch
 */
// ROS
#include <tf/transform_datatypes.h>

// RS
#include <robosherlock/conversion/conversion.h>
#include <robosherlock/types/tf_types.h>
#include <robosherlock/types/all_types.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/DrawerDetectionData.h>

namespace suturo_perception {
    namespace conversion {
        template<typename T>
        void from(const uima::FeatureStructure &fs, T &output);

        template<typename T>
        //hier nicht
        uima::FeatureStructure to(uima::CAS &cas, const T &input);

        u_int decode_shape(std::vector<rs::Shape> shapes);

        void makeObjectDetectionData(geometry_msgs::PoseStamped &pose, rs::Geometry &geometry, u_int shape,
                                     std::string region, std::string &objClass, float confidence,
                                     std_msgs::ColorRGBA &c, suturo_perception_msgs::ColorHSV &hsv,
                                     suturo_perception_msgs::ObjectDetectionData &odd);

        void makeTableObjectDetectionData(geometry_msgs::PoseStamped &pose, rs::Geometry &geometry,
                                     std::string region, std::string &objClass, float confidence,
                                     suturo_perception_msgs::ObjectDetectionData &odd);

        void makeDrawerDetectionData(geometry_msgs::PoseStamped &pose, rs::Geometry &geometry,
                                     std::string region, std::string &objClass, float confidence,
                                     suturo_perception_msgs::DrawerDetectionData &ddd);
    }
}
