//
// Created by tangchu on 2017/9/12.
//

#include "AlgoTensorflowPersonDetector.h"
#include "classes.h"
using namespace ninebot_algo;
using namespace ninebot_tensorflow;

//using namespace tensorflow;

AlgoTensorflowPersonDetector::AlgoTensorflowPersonDetector(RawData *rawInterface, int run_sleep_ms, bool is_render)
        :AlgoBase(rawInterface,run_sleep_ms)
{
    ALOGD("tensorflow: start init");
    canvas = cv::Mat::zeros(cv::Size(640, 360), CV_8UC3);
    main_RawData = mRawDataInterface;
    this->m_isRender = is_render;

    // Person detector: must be uploaded to Loomo. Can detect more than people but we filter out the rest
    std::string graph_path = "/sdcard/tensorflow/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb";

    tensorflow::ConfigProto& config = options.config;
    //set to single thread
    config.set_intra_op_parallelism_threads(1);

    session = NewSession(options);
    graph_definition = new tensorflow::GraphDef();
    tensorflow::Status s = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, graph_definition);
    if (!s.ok()) {
        ALOGD("tensorflow: Failed to create load Graph from path: %s", graph_path.c_str());
        return;
    }
    if(session != NULL)
    {
        s = session->Create(*graph_definition);
    }
    else
    {
        ALOGD("tensorflow: session == NULL");
        return;
    }

    if (!s.ok()) {
        ALOGD("tensorflow: Could not create TensorFlow Session: %s", s.error_message().c_str());
        return;
    }

    //initialize input tensor
    img_tensor = new tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, IMG_W, IMG_H, 3}));
    //stepnb = 0;
}

AlgoTensorflowPersonDetector::~AlgoTensorflowPersonDetector()
{
    delete img_tensor;
    img_tensor = NULL;
    delete graph_definition;
    graph_definition = NULL;
    delete session;
    session = NULL;
}

bool AlgoTensorflowPersonDetector::convert_image_to_feed_data(const cv::Mat srcImg, tensorflow::Tensor* imgTensor)
{
    if(srcImg.empty() || imgTensor == NULL || srcImg.channels() != IMG_C)
        return false;

    cv::Mat resizedImg;
    cv::resize(srcImg, resizedImg, cv::Size(IMG_W, IMG_H));
    cv::Mat float_img;
    resizedImg.convertTo(float_img, CV_32F);

    if(IMG_VALUE_NORMALIZATION)
    {
        float_img = float_img * 0.003922;//1 / 255
    }

    std::vector<cv::Mat> splited_mat;
    cv::split(float_img, splited_mat);
    cv::Mat r_channel = splited_mat[2].clone();
    cv::Mat g_channel = splited_mat[1].clone();
    cv::Mat b_channel = splited_mat[0].clone();

    if(IMG_ZERO_MEAN)
    {
        if(IMG_VALUE_NORMALIZATION)
        {
            r_channel = r_channel - R_MEAN * 0.003922;// 1 / 255
            g_channel = g_channel - G_MEAN * 0.003922;
            b_channel = b_channel - B_MEAN * 0.003922;
        }
        else
        {
            r_channel = r_channel - R_MEAN;
            g_channel = g_channel - G_MEAN;
            b_channel = b_channel - B_MEAN;
        }
    }
    float* data = const_cast<float *>(reinterpret_cast<const float *>(img_tensor->tensor_data().data()));

    for(int i = 0; i < IMG_H; i ++)
    {
        for(int j = 0; j < IMG_W; j ++)
        {
            data[(i * IMG_W + j) * 3] = r_channel.at<float>(i, j);
            data[(i * IMG_W + j) * 3 + 1] = g_channel.at<float>(i, j);
            data[(i * IMG_W + j) * 3 + 2] = b_channel.at<float>(i, j);
        }
    }
    return true;
}

bool AlgoTensorflowPersonDetector::step()
{
    ALOGD("tensorflow: start step");
    auto start = std::chrono::high_resolution_clock::now();
    main_RawData->retrieveColor(raw_color);
    //main_RawData->retrieveDS4Color(raw_color, false);
    if (raw_color.timestampSys == 0)
    {
        return false;
    }
    if(raw_color.image.channels() != 3)
    {
        ALOGD("tensorflow: not supported!");
        return false;
    }
    cv::Mat tcolor = raw_color.image.clone();
    cv::resize(tcolor, tcolor, cv::Size(640,480));

    vector<pair<string, tensorflow::Tensor>> inputs = {{"input", *img_tensor}};
    std::vector<tensorflow::Tensor> outputs;


    int inputHeight = tcolor.size().height; // Height & width of input image
    int inputWidth = tcolor.size().width;

    tensorflow::Tensor imgTensorWithSharedData(tensorflow::DT_UINT8, {1, inputHeight, inputWidth, tcolor.channels()});
    uint8_t *p = imgTensorWithSharedData.flat<uint8_t>().data();
    cv::Mat outputImg(inputHeight, inputWidth, CV_8UC3, p);
    tcolor.convertTo(outputImg, CV_8UC3);

    convert_image_to_feed_data(tcolor, img_tensor);

    tensorflow::Status s = session->Run({{"image_tensor:0", imgTensorWithSharedData}},
                                                       {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"},
                                                       {},
                                                       &outputs);
    if (!s.ok()){
        ALOGD("tensorflow: Failed to run session: %s", s.error_message().c_str());
        return -1;
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end-start;
    {
        std::lock_guard<std::mutex> lock(mMutexTimer);
        m_ptime = elapsed.count()*0.5 + m_ptime*0.5;
    }

    //show result
    cv::Mat show_mat;
    cv::resize(tcolor,show_mat,cv::Size(480,360));
    cv::Mat draw_mat = canvas(cv::Rect(0,0,480,360));
    show_mat.copyTo(draw_mat);

    // Draw boxes around the people
    draw_boxes(outputs, draw_mat);

    /*vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    std::string folder = "/sdcard/tensorflow/ssd_mobilenet_v1_coco_2017_11_17/image";
    std::stringstream stepnb_str;
    stepnb_str << stepnb;

    cv::imwrite( folder + stepnb_str.str() + ".png", draw_mat, compression_params);
    stepnb += 1;*/

    return true;
}

float AlgoTensorflowPersonDetector::runTime()
{
    std::lock_guard<std::mutex> lock(mMutexTimer);
    return m_ptime;
}

bool AlgoTensorflowPersonDetector::showScreen(void* pixels)
{
    if(m_isRender)
    {
        cv::Mat display_img;
        {
            std::lock_guard <std::mutex> lock(_mutex_display);
            if (canvas.empty())
                return false;
            cv::cvtColor(canvas, display_img, CV_BGR2RGBA);
        }
        memcpy(pixels, (void *)display_img.data, display_img.cols * display_img.rows * 4);
    }
    return true;
}

// Function that draws boxes on input image around the people detected
void AlgoTensorflowPersonDetector::draw_boxes(std::vector<tensorflow::Tensor>& output_vector, cv::Mat& input_image)
{
    // Get the list of scores & classes associated as well as the number of detections and the size
    // of objects detected
    tensorflow::TTypes<float>::Flat scores = output_vector[1].flat<float>();
    tensorflow::TTypes<float>::Flat classes = output_vector[2].flat<float>();
    tensorflow::TTypes<float>::Flat num_detections = output_vector[3].flat<float>();
    auto boxes = output_vector[0].flat_outer_dims<float,3>();

    int detectionsCount = (int)(num_detections(0));
    int drawnDetections = 0;
    cv::RNG rng(12345);

    // Loop through all detections
    for(int i = 0; i < detectionsCount; i++)
    {
        ALOGD("tensorflow: Detection nb %i, the score is %lf\n", i, scores(i));

        // If probability of detection > 0.5 &&  detection class == person, draw box on image
        // Because the network detects more than just people
        if(scores(i) > 0.5 && classes(i) == 1) {
            //float boxClass = classes(i);

            // Upper left corner of detection
            float x1 = float(input_image.size().width) * boxes(0,i,1);
            float y1 = float(input_image.size().height) * boxes(0,i,0);

            // Bottom right corner of detection
            float x2 = float(input_image.size().width) * boxes(0,i,3);
            float y2 = float(input_image.size().height) * boxes(0,i,2);


            // Create new color for the detection
            cv::Scalar randomColor = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            // Draw detection box on input image
            cv::rectangle(input_image, cv::Point(x1, y1), cv::Point(x2, y2), randomColor);

            drawnDetections++;
        }

    }
}