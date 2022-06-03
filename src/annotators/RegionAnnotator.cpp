#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <suturo_perception/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/io/TFListenerProxy.h>
#include <robosherlock/utils/exception.h>

#include <visualization_msgs/Marker.h>

using namespace uima;


class RegionAnnotator : public Annotator
{
    struct SemanticMapItem
    {
        tf::Transform transform;
        std::string reference_frame;
        std::string name, type;
        double y_dimension, z_dimension, x_dimension;
        cv::Vec4f plane_eq;
        bool has_plane_equations = false;
    };

    std::string marker_topic_;
    ros::Publisher marker_publisher_;
    rs::TFListenerProxy listener_;

    void publishSemanticMapMarker(const SemanticMapItem &item) {
        visualization_msgs::Marker m, m_text;
        m.header.frame_id = m_text.header.frame_id = item.reference_frame;
        m.header.stamp = m_text.header.stamp = ros::Time::now();
        m_text.text = item.name;

        m.ns = m_text.ns = item.name;
        m.id = 0;
        m_text.id = 1;
        m.lifetime = m_text.lifetime = ros::Duration();

        m.type = visualization_msgs::Marker::CUBE;
        m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.color.r = 0.0f;
        m.color.g = 0.0f;
        m.color.b = 1.0f;
        m.color.a = 0.75f;
        m_text.color.a = 1;

        auto transform = item.transform;
        auto origin = transform.getOrigin();
        auto rotation = transform.getRotation();

        m.pose.position.x = m_text.pose.position.x = origin.x();
        m.pose.position.y = m_text.pose.position.y = origin.y();
        m.pose.position.z = m_text.pose.position.z = origin.z();
        m.pose.orientation.x = rotation.getX();
        m.pose.orientation.y = rotation.getY();
        m.pose.orientation.z = rotation.getZ();
        m.pose.orientation.w = rotation.getW();
        m_text.pose.orientation.w = 1;

        m.scale.x = item.x_dimension;
        m.scale.y = item.y_dimension;
        m.scale.z = item.z_dimension;
        m_text.scale.z = 0.1f;

        marker_publisher_.publish(m);
        marker_publisher_.publish(m_text);
    }

public:
    RegionAnnotator()
    : marker_topic_("perception_marker/annotatedRegions")
    {
    }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
      ros::NodeHandle n;
    marker_publisher_ = n.advertise<visualization_msgs::Marker>("perception_marker/annotatedRegions", 100);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process started");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    outInfo("Looking for clusters");

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);

    outInfo("Processing " << clusters.size() << " clusters");

    for (auto &cluster : clusters) {
      std::vector<rs::PoseAnnotation> poses;
      cluster.annotations.filter(poses);

      if(!poses.empty()) {
        rs::StampedPose pose = poses[0].world();
        std::vector<rs::SemanticMapObject> regions;
          cas.get(VIEW_SEMANTIC_MAP, regions);

	outInfo("Found " << regions.size() << " regions");
          tf::StampedTransform camToWorld;

          camToWorld.setIdentity();
          if (scene.viewPoint.has())
          {
              rs::conversion::from(scene.viewPoint.get(), camToWorld);
          }
          else
          {
              outWarn("No camera to world transformation, no further processing!");
              throw rs::FrameFilterException();
          }

        for(rs::SemanticMapObject region : regions) {
            SemanticMapItem item;
            item.name = region.name();
            item.reference_frame = region.typeName();
            item.y_dimension = region.width();
            item.z_dimension = region.height();
            item.x_dimension = region.depth();
            //todo reference frame fehlt
            tf::Transform transformus;
            rs::conversion::from(region.transform(), transformus);
            item.transform = transformus; //todo change name
            if(item.reference_frame != "map") {
                tf::StampedTransform regionToWorld;

                listener_.listener->waitForTransform("map", item.reference_frame,
                                                   ros::Time(0),
                                                   ros::Duration(2.0));
                listener_.listener->lookupTransform("map", item.reference_frame, ros::Time(0),
                                                    regionToWorld);
                item.transform = regionToWorld * item.transform; //todo find out
            }
            tf::Transform transform;
            transform = item.transform.inverse() * camToWorld;

            //TODO umrechnung Region frames einfügen
          auto regionPos = transform;
          auto clusterPos = pose.translation();
          double x = std::abs(regionPos.getOrigin().x() - clusterPos[0]);
          double y = std::abs(regionPos.getOrigin().y() - clusterPos[1]);
          double z = std::abs(regionPos.getOrigin().z() - clusterPos[2]);

//          outInfo(region.name() << " Origin: " << regionPos[0]<<","<<regionPos[1]<<","<<regionPos[2]);
//          outInfo("Cluster Origin: " << clusterPos[0]<<","<<clusterPos[1]<<","<<clusterPos[2]);
//          outInfo("Point: " << x << "," << y << "," << z);
//          outInfo("Region Frustum: " << region.height()/2 <<","<< region.width()/2 <<","<< region.depth()/2);

          if(z < item.z_dimension/2 && x < item.x_dimension/2 && y < item.y_dimension/2) {
              suturo_perception::Region r = rs::create<suturo_perception::Region>(tcas);
              r.name.set(item.name);
              cluster.annotations.append(r);
              outInfo("Added region annotation!");
              break;
          }
        }
      }
      else {
          outInfo("No pose annotated!");
      }
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RegionAnnotator)
