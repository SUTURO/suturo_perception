/**
 * Modified RoboSherlock Process Manager
 * Optimized for the Suturo Perception Pipelines
 * @author Fenja Kollasch & Vanessa Hassouna
 */
#include <suturo_perception/SuturoProcessManager.h>
#include <std_msgs/ColorRGBA.h>

SuturoProcessManager::SuturoProcessManager(ros::NodeHandle n, const std::string savePath, std::string &name) :
    savePath(savePath),
    nh(n),
    image_transport(nh),
    visualizer(false, false),
    name(name)
{
    setup();
    //error: no macthing  member function  for call  to 'advertise service'
    vis_service = nh.advertiseService("vis_command", &SuturoProcessManager::visControlCallback, this);
    image_pub = image_transport.advertise("result_image", 1, true);

    visualize = true;
}

SuturoProcessManager::SuturoProcessManager(ros::NodeHandle n, std::string &name) :
        nh(n),
        image_transport(nh),
        visualizer(false, false),
        name(name)
{
    setup();

    visualize = name.compare("hsrb_1ms") == 0;
}

void SuturoProcessManager::setup() {
    outInfo("A RoboSherlock process manager optimized for the Suturo perception was created.");
    signal(SIGINT, RSProcessManager::signalHandler);

    outInfo("Creating resource manager");
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance((name+"/RoboSherlock").c_str());
    resourceManager.setLoggingLevel(uima::LogStream::EnError);

    regions = std::vector<std::string>();
}

void SuturoProcessManager::init(std::string &pipeline) {
    outInfo("Initializing Engine...");
    signal(SIGINT, RSProcessManager::signalHandler);

    std::string pipelinePath;
    rs::common::getAEPaths(pipeline, pipelinePath);
    engine = rs::createRSAggregateAnalysisEngine(pipelinePath);

    uima::ErrorInfo errorInfo;
    mongo::client::GlobalInstance instance;

    if(visualize) {
        // visualizer.addVisualizableGroupManager(engine->getAAEName());
        // Use legacy visualisation
        visualizer.start();
    }
}

void SuturoProcessManager::run(std::map<std::string, boost::any> args, std::vector<ObjectDetectionData>& detectionData) {
    outInfo("Running the Suturo Process Manager");
    outInfo("Setting up runtime parameters...");

    if(args.find("regions") != args.end()) {
        regions = boost::any_cast<std::vector<std::string>>(args["regions"]);

        if(regions.size() > 0 && regions[0].compare("robocup_default") == 0) {
            outInfo("RegionFilter will be disabled..");

            filter_regions = false;
        }
        else {
            outInfo("Custom regions are displayed");
            filter_regions = true;
        }
    }
    else {
	outInfo("Custom regions are disabled");
        regions = std::vector<std::string>();
        filter_regions = false;
    }

    engine->overwriteParam("SuturoRegionFilter", "enabled", filter_regions);
    engine->iv_annotatorMgr.iv_vecEntries[engine->getIndexOfAnnotator("SuturoRegionFilter")].iv_pEngine->reconfigure();

    outInfo("Analysis engine starts processing");
    engine->processOnce();
    uima::CAS* tcas = engine->getCas();
    rs::SceneCas cas(*tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);

    for (auto &cluster : clusters) {
        std::vector<suturo_perception::Region> region;
        cluster.annotations.filter(region);

        if(region.empty()) {
            getClusterFeatures(cluster, detectionData);
        }
        else {
            outInfo("Cluster region: " << region[0].name());

            if(!filter_regions || std::find(regions.begin(), regions.end(), region[0].name()) != regions.end()) {
                getClusterFeatures(cluster, detectionData);
            }
            else {
                outInfo("Object was ignored because it seems to be placed on the wrong surface");
            }
        }
    }

    tcas->reset();
}

bool SuturoProcessManager::has_vertical_plane() {
    outInfo("Looking for a vertical plane...");
    outInfo("Running the analysis engine...");
    engine->processOnce();
    uima::CAS* tcas = engine->getCas();
    rs::SceneCas cas(*tcas);
    rs::Scene scene = cas.getScene();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    return !planes.empty();
}


void SuturoProcessManager::getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data) {

    std::vector<rs::Geometry> geometry;
    cluster.annotations.filter(geometry);

    if(!geometry.empty()) {
        std::vector<rs::PoseAnnotation> poses;
        cluster.annotations.filter(poses);

        std::vector<rs::Classification> classification;
        cluster.annotations.filter(classification);

        std::vector<rs::ClassConfidence> confi;
        cluster.annotations.filter(confi);

        std::vector<rs::Shape> shapes;
        cluster.annotations.filter(shapes);

        std::vector<rs::SemanticColor> colorH;
        cluster.annotations.filter(colorH);

        std::vector<rs::ColorHistogram> histogram;
        cluster.annotations.filter(histogram);

        geometry_msgs::PoseStamped poseStamped;
        std::string objClass;
        std::string knownObjClass;
        float confidence = 0;
        float knownObjConfidence = 0;
        float r = 0.0;
        float g = 0.0;
        float b = 0.0;
        std_msgs::ColorRGBA c;
        suturo_perception_msgs::ColorHSV hsv;

        ObjectDetectionData odd;

        if(!colorH.empty()) {
            std::string tmp_color_string = colorH[0].color.get();
            outInfo("" << colorH[0].color.get());


            if(tmp_color_string == "red") {
                r = 255.0;}
            if(tmp_color_string == "yellow") {
                r = 255.0;
                g = 255.0;}
            if(tmp_color_string == "green") {
                g = 255.0;}
            if(tmp_color_string == "cyan") {
                g = 255.0;
                b = 255.0;}
            if(tmp_color_string == "blue") {
                b = 255.0;}
            if(tmp_color_string == "magenta") {
                r = 255.0;
                b = 255.0;}
            if(tmp_color_string == "white") {
                r = 255.0;
                g = 255.0;
                b = 255.0;}
            if(tmp_color_string == "grey") {
                r = 127.0;
                g = 127.0;
                b = 127.0;}

            c.r = r;
            c.g = g;
            c.b = b;
            c.a = 1;

        } else {
            ROS_WARN("Warning: No color was perceived.");
        }

        if(!histogram.empty()) {
            cv::Mat hist;
            rs::conversion::from(histogram[0].hist.get(), hist);
            const cv::Vec3b *itHSV = hist.ptr<cv::Vec3b>();

            const uint8_t hue = itHSV->val[0] * 360 / 255;
            const uint8_t sat = itHSV->val[1] * 100 / 255;
            const uint8_t val = itHSV->val[2] * 100 / 255;

            hsv.h = hue;
            hsv.s = sat;
            hsv.v = val;

            auto pos = cluster.rois.get().roi.get().pos.get();
            ROS_ERROR_STREAM(
                    "HSV data: x=" << pos.x.get() << ", " << pos.y.get() << " HSV( " << hsv.h << ", " << hsv.s << ", "
                                   << hsv.v << " )");

        } else {
            ROS_WARN("No color histogram annotated.");
        }

        if(!poses.empty()) {
            suturo_perception::conversion::from(poses[0].world.get(), poseStamped);
        } else {
            ROS_WARN("Warning: No pose information was perceived.");
        }

        if(!classification.empty()){
            objClass = classification[0].classname.get();
            //knownObjClass = classification[1].classname.get();
            outInfo("OBJCLASSNAME >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: " << classification[0].classname.get());
            //outInfo("OBJCLASSNAME >>>>>>>>>KNOWN>>>>>>>KNOWN>>>>>>>: " << classification[1].classname.get());

            if(!confi.empty()){
                confidence = confi[0].score.get();
                //knownObjConfidence = confi[1].score.get();
                outInfo("OBJCLASSCONFI <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<: " << confi[0].score.get());
                //outInfo("OBJCLASSCONFI <<<<<<<<<KNOWN<<<<<<<KNOWN<<<<<<: " << confi[1].score.get());

                /*if(confidence < knownObjConfidence){
                    objClass = knownObjClass;
                }*/
            } else {
                ROS_WARN("Warning: No confidence was perceived.");
            }

        } else {
            ROS_WARN("Warning: No object class was perceived.");
        }

        std::vector<suturo_perception::Region> region;
        cluster.annotations.filter(region);

        if(region.empty()) {
            ROS_WARN("Warning: No region annotated.");

            suturo_perception::conversion::makeObjectDetectionData(poseStamped, geometry[0],
                                                                   suturo_perception::conversion::decode_shape(shapes), "", objClass, confidence, c, hsv, odd);
        }
        else {
            suturo_perception::conversion::makeObjectDetectionData(poseStamped, geometry[0],
                                                                   suturo_perception::conversion::decode_shape(shapes), region[0].name(), objClass, confidence, c, hsv, odd);
        }


        data.push_back(odd);

    } else {
        ROS_WARN("Warning: Object Feature detection was unsuccessful. No geometries were recognized for this object.");
    }

}

bool SuturoProcessManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
                                          robosherlock_msgs::RSVisControl::Response &res)
{
    std::string command = req.command;
    bool result = true;
    std::string activeAnnotator = "";
    if(command == "next")
    {
        activeAnnotator = visualizer.nextAnnotator();

    }
    else if(command == "previous")
    {
        activeAnnotator = visualizer.prevAnnotator();
    }
    else if(command != "")
    {
        activeAnnotator = visualizer.selectAnnotator(command);
    }
    if(activeAnnotator == "")
        result = false;

    res.success = result;

    res.active_annotator = activeAnnotator;
    return result;
}
