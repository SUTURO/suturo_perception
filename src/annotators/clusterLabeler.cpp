#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/DrawingAnnotator.h>

#include <thread>
#include <iostream>

using namespace uima;


class clusterLabeler : public DrawingAnnotator
{
private:
    //These are fields used for the GUI. Comment in if you want to use it.
    //int argc = 0;
    //char **argv = NULL;
    //guiWrapper* wrapper;
    //std::thread gui;

    //CV Matrix to hold an image
    cv::Mat current_image;

public:

  clusterLabeler(): DrawingAnnotator(__func__)
  {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
      outInfo("initialize");
      //Initializes a new guiWrapper and runs the GUI application in new thread.
      //wrapper = new guiWrapper(argc, argv);
      //gui = std::thread(&guiWrapper::run, wrapper);
      //outInfo("spawned window");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
      //Initializing the CAS
      outInfo("Starting Cluster to number mapping.");
      rs::StopWatch clock;
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();

      //Get the current image from the cas
      cas.get(VIEW_COLOR_IMAGE, current_image);

      //Get the clusters in the image found by previous annotators
      std::vector<rs::ObjectHypothesis> clusters;
      scene.identifiables.filter(clusters);
      outInfo("Found " << clusters.size() << " clusters.");

      //create iterator to iterate through cluster vector
      std::vector<rs::ObjectHypothesis>::iterator cluster;

      //Using the iterator to display the cluster numbering
      for (cluster = clusters.begin(); cluster < clusters.end(); cluster++) {
          //set region of interest for the cluster
          rs::ImageROI cluster_roi = cluster->rois.get();
          //create Rectangle
          cv::Rect rectangle;
          rs::conversion::from(cluster_roi.roi.get(), rectangle);
          //Draw the numbering for the cluster
          drawCluster(current_image, rectangle, std::to_string((cluster - clusters.begin())));
          //Here the visualizing method of the guiWrapper would be called.
      }

      outInfo("took: " << clock.getTime() << " ms.");
      return UIMA_ERR_NONE;
  }
  /*
   * Draws the rectangle into the image and labels it with the number.
   */
  static void drawCluster(cv::Mat input, cv::Rect rectangle, const std::string &label){
      std::stringstream ss;
      ss<<label;
      std::string text = ss.str();
      cv::rectangle(input, rectangle, CV_RGB(0, 255, 0), 2);
      int offset = -3;
      int baseLine;
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1.5, 2.0, &baseLine);
      cv::putText(input, text, cv::Point(rectangle.x + rectangle.width, rectangle.y - offset - textSize.height), cv::FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0, 0, 0), 2.0);
  }

  void drawImageWithLock(cv::Mat &disp){
        disp = current_image.clone();
  }


};

// This macro exports an entry point that is used to create the annotators.
MAKE_AE(clusterLabeler)
