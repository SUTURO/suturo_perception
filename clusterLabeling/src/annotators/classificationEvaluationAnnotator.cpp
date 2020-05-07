#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <stdlib.h>
#include <string>

using namespace uima;


class classificationEvaluationAnnotator : public Annotator
{
private:
  float test_param;
    cv::Mat current_image;
  rapidjson::Document document;

  int framecounter = 0;
  int correct_samples = 0;
  int total_samples = 0;
  float accuracy;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    std::ifstream file("/home/janf/suturo/src/clusterLabeling/labeling/table_view_1_labeling.json");
    std::string content(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    document.Parse(content.c_str());

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
      //Initializing the CAS
      outInfo("Starting classification Evaluation");
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();

      //Get the current image from the cas
      cas.get(VIEW_COLOR_IMAGE, current_image);

      //Get the clusters in the image found by previous annotators
      std::vector<rs::ObjectHypothesis> clusters;
      scene.identifiables.filter(clusters);
      outInfo("Found " << clusters.size() << " clusters.");

      outInfo("Frame: " << framecounter);
      if (!document["clusterLabeling"][framecounter]["clusters"].Empty()) {

          for (int i = 0; i < clusters.size(); i++) {
              std::vector<rs::Classification> classResult;
              clusters[i].annotations.filter(classResult);
              outInfo("Class result size is: " << classResult.size());
              if (classResult.size() > 0) {
                  outInfo("Comparing " << classResult[0].classname.get() << " and "
                                       << document["clusterLabeling"][framecounter]["clusters"][i]["label"].GetString());

                  if (classResult[0].classname.get().compare(
                          document["clusterLabeling"][framecounter]["clusters"][i]["label"].GetString()) == 0) {
                      outInfo("Correct classification.");
                      correct_samples++;
                      total_samples++;
                  } else {
                      outInfo("Incorrect classification.");
                      total_samples++;
                  }
              } else {
                  total_samples++;
              }

          }
          accuracy = ((float) correct_samples) / ((float) total_samples);
          outInfo("Current accuracy is " << (accuracy * 100) << "%.");
      }
      framecounter++;

      return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotators.
MAKE_AE(classificationEvaluationAnnotator)