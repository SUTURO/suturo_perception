/***
 * Author: Evan Kapitzke
 *
 * Created for SUTURO Bachelorproject based on SacModelAnnotator (RoboSherlock)
 */
#include <uima/api.hpp>
#include <utility>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
// RS
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/types/all_types.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/utils/time.h>

// Debug
#define DEBUG

using namespace uima;


class SuturoShapeAnnotator : public DrawingAnnotator
{
private:
  // SAC Parameters:
  int param_max_iterations = 1000;
  float param_distance_threshold = 0.03;
  float param_distance_weight = 0.1;
  float param_radius_min = 0;
  float param_radius_max = 0.1;
  float param_confidence_min = 0.5;
  float param_eps_angle = 0.1;

  // Visualisation:
  cv::Mat annotatorView;


  /***
   * Draws the result
   * @param rect Rectangle
   * @param result_name Annotated class name
   * @param confidence Result confidence
   * @param color Rectangle- and text color
   */
  void drawResult(cv::Rect &rect, std::string &result_name, float confidence, cv::Scalar color){
#pragma omp critical
      {
        cv::rectangle(annotatorView, rect, color, 2);
        cv::putText(annotatorView, result_name, cv::Point(rect.x, rect.y - 10),
              cv::FONT_HERSHEY_COMPLEX, 0.8, color, 1);
        cv::putText(annotatorView, std::to_string(confidence), cv::Point(rect.x, rect.y + rect.height + 20),
                  cv::FONT_HERSHEY_COMPLEX, 0.8, color, 1);
      }
  }

  /***
   * Tries to find a plane and a erpendicular plane in the given cluster.
   * Annotates the found results.
   * @param tcas CAS
   * @param clusterCloud Point cloud
   * @param clusterNormals Normals
   */
  float annotateBox(CAS &tcas, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &clusterCloud,
                          pcl::PointCloud<pcl::Normal>::Ptr &clusterNormals)
  {
      pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);

      // Init SAC:
      pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(param_distance_weight);
      seg.setMaxIterations(param_max_iterations);
      seg.setDistanceThreshold(param_distance_threshold);
      seg.setEpsAngle(param_eps_angle * (M_PI / 180.0f));
      seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
      seg.setInputCloud(clusterCloud);
      seg.setInputNormals(clusterNormals);

      // Segment and add relative number of inliers as confidence:
      seg.segment(*planeInliers, *coefficients_plane);

#ifdef DEBUG
      outInfo("Size of cluster: " << clusterCloud->width);
      outInfo("Number of BOX inliers: " << planeInliers->indices.size());
#endif

      if(planeInliers->indices.empty())
      {
          return 0;
      }
      return (float)planeInliers->indices.size() / clusterCloud->width;
  }

  /***
   * Tries to fit a cylinder model on the given cluster
   * and annotates it.
   * @param tcas CAS
   * @param clusterCloud Point cloud
   * @param clusterNormals Normals
   */
  float annotateCylinder(CAS &tcas, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &clusterCloud,
                               pcl::PointCloud<pcl::Normal>::Ptr &clusterNormals)
  {
      pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr cylinderInliers(new pcl::PointIndices);

      // Init SAC:
      pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(param_distance_weight);
      seg.setMaxIterations(param_max_iterations);
      seg.setDistanceThreshold(param_distance_threshold);
      seg.setRadiusLimits(param_radius_min, param_radius_max);
      seg.setInputCloud(clusterCloud);
      seg.setInputNormals(clusterNormals);

      // Segment and add relative number of inliers as confidence:
      seg.segment(*cylinderInliers, *coefficients_cylinder);

#ifdef DEBUG
      outInfo("Size of cluster: " << clusterCloud->width);
      outInfo("Number of CYLINDER inliers: " << cylinderInliers->indices.size());
#endif

      if(cylinderInliers->indices.empty())
      {
          return 0;
      }
      return (float)cylinderInliers->indices.size() / clusterCloud->width;
  }

  /***
   * Tries to fit a sphere model on the given cluster
   * and annotates it.
   * @param tcas CAS
   * @param clusterCloud Point cloud
   * @param clusterNormals Normals
   */
  float annotateSphere(CAS &tcas, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &clusterCloud,
                             pcl::PointCloud<pcl::Normal>::Ptr &clusterNormals)
  {
      pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr sphereInliers(new pcl::PointIndices);

      // Init SAC:
      pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(param_distance_weight);
      seg.setMaxIterations(param_max_iterations);
      seg.setDistanceThreshold(param_distance_threshold);
      seg.setRadiusLimits(param_radius_min, param_radius_max);
      seg.setInputCloud(clusterCloud);
      seg.setInputNormals(clusterNormals);
      seg.setEpsAngle(param_eps_angle * (M_PI / 180.0f));

      // Segment and add relative number of inliers as confidence:
      seg.segment(*sphereInliers, *coefficients_sphere);

#ifdef DEBUG
      outInfo("Size of cluster: " << clusterCloud->width);
      outInfo("Number of SPHERE inliers: " << sphereInliers->indices.size());
#endif

      if(sphereInliers->indices.empty())
      {
          return 0;
      }
      return (float)sphereInliers->indices.size() / clusterCloud->width;
  }


public:

  SuturoShapeAnnotator() : DrawingAnnotator(__func__) {
  }

  TyErrorId initialize(AnnotatorContext &ctx) override
  {
    outInfo("initialize");

    ctx.extractValue("max_iterations", param_max_iterations);
    ctx.extractValue("distance_threshold", param_distance_threshold);
    ctx.extractValue("distance_weight", param_distance_weight);
    ctx.extractValue("radius_min", param_radius_min);
    ctx.extractValue("radius_max", param_radius_max);
    ctx.extractValue("confidence_min", param_confidence_min);
    ctx.extractValue("eps_angle", param_eps_angle);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy() override
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) override
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);

    cas.get(VIEW_CLOUD,*cloud_ptr);
    cas.get(VIEW_NORMALS, *normals_ptr);
    cas.get(VIEW_COLOR_IMAGE, annotatorView);

    // Print Parameters:
    outInfo("Parameters:\n" <<
            "\nmax_iterations: " << param_max_iterations <<
            "\ndistance_threshold: " << param_distance_threshold <<
            "\ndistance_weight: " << param_distance_weight <<
            "\nradius_min: " << param_radius_min <<
            "\nradius_max: " << param_radius_max <<
            "\nconfidence_min: " << param_confidence_min);

    // Process clusters:
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    outInfo("Processing " << clusters.size() << " point clusters");

#pragma omp parallel for
    for(int idx = 0; idx < clusters.size(); ++idx) {
        // Convert cluster, save normals:
        pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::Normal>::Ptr clusterNormals(new pcl::PointCloud<pcl::Normal>);

        rs::conversion::from(((rs::ReferenceClusterPoints)clusters[idx].points.get()).indices(), *clusterIndices);
        for(int & indice : clusterIndices->indices)
        {
            clusterCloud->points.push_back(cloud_ptr->points[indice]);
            clusterNormals->points.push_back(normals_ptr->points[indice]);
        }

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        clusterNormals->width = normals_ptr->points.size();
        clusterNormals->height = 1;
        clusterNormals->is_dense = true;

        // Process cloud:
        std::string result_name = "none";
        float result_confidence = 0;
        cv::Scalar result_color;

        float boxConfidence = annotateBox(tcas, clusterCloud, clusterNormals);
        float cylinderConfidence = annotateCylinder(tcas, clusterCloud, clusterNormals);
        float sphereConfidence = annotateSphere(tcas, clusterCloud, clusterNormals);

        // Select shape with highest confidence:
        if(boxConfidence > cylinderConfidence && boxConfidence > sphereConfidence)
        {
            // It's a box:
            result_color = cv::Scalar(235, 52, 52);
            result_confidence = boxConfidence;
            result_name = "box";
        }
        else if(cylinderConfidence > boxConfidence && cylinderConfidence > sphereConfidence)
        {
            // It's a cylinder:
            result_color = cv::Scalar(52, 235, 82);
            result_confidence = cylinderConfidence;
            result_name = "cylinder";
        }
        else
        {
            // It's a sphere:
            result_color = cv::Scalar(52, 76, 235);
            result_confidence = sphereConfidence;
            result_name = "sphere";
        }

        // Check confidence:
        if(result_confidence >= param_confidence_min)
        {
            // Add annotation with highest confidence:
            rs::Shape result = rs::create<rs::Shape>(tcas);
            result.shape.set(result_name);
            result.confidence.set(result_confidence);
            clusters[idx].annotations.append(result);
        }
        else
        {
            // Confidence is to low:
            result_color = cv::Scalar(130, 130, 130);
        }

        // Visualisation:
        rs::ImageROI imageRoi(clusters[idx].rois());
        cv::Rect rect;
        rs::conversion::from(imageRoi.roi(), rect);
        drawResult(rect, result_name, result_confidence, result_color);

#ifdef DEBUG
        outInfo("Selected shape: " << result_name << " (confidence: " << result_confidence <<")");
#endif

    }

    // Print time:
    outInfo("took: " << clock.getTime() << " ms.");

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp) override
  {
    disp = annotatorView.clone();
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SuturoShapeAnnotator)