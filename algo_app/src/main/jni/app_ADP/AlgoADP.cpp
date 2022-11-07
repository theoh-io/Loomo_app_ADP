#include "AlgoADP.h"
#include "ninebot_log.h"
#include "AlgoUtils.h"
#include "Quaternion.h"
#include "Vector3.h"

#include <thread>
#include <mutex>
#include <algorithm>
#include "SocketServer.h"

#define PI 3.1415

namespace ninebot_algo
{
    namespace adp_algo
    {
        using namespace std;
        using namespace cv;

        AlgoADP::AlgoADP(RawData *rawInterface, int run_sleep_ms, bool isRender)
                :AlgoBase(rawInterface,run_sleep_ms,true)
        {
            head_pitch= 0;
            head_yaw= -1.5;
            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadPos(head_yaw, head_pitch, 0);
            m_is_init_succed = false;
            m_ptime = 1;
            m_isRender = isRender;
            pose_isRecording = false;
            canvas = cv::Mat::zeros( cv::Size(640, 360), CV_8UC3 );
            render_depth=true; // if true will reduce size of camera rendering and render depth on the side
            text_pos=400;
            m_safety_control = true;
            m_is_track_head = false;
            m_is_track_vehicle = false;
            m_p_local_mapping = NULL;
            m_target_distance = -1.0f;
            m_target_theta = 0.0f;
            init();
        }

        AlgoADP::~AlgoADP() {
            this->stopPoseRecord();

            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadPos(head_yaw, head_pitch, 0);

            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }

            if(m_p_localmapping_rawdata != NULL) {
                delete m_p_localmapping_rawdata;
                m_p_localmapping_rawdata = NULL;
            }

            if(m_p_server_control != NULL) {
                delete m_p_server_control;
                m_p_server_control = NULL;
            }

            if(m_p_server_perception != NULL) {
                delete m_p_server_perception;
                m_p_server_perception = NULL;
            }

            if(m_p_server_estimation != NULL) {
                delete m_p_server_estimation;
                m_p_server_estimation = NULL;
            }

            if(m_p_server_mapping != NULL) {
                delete m_p_server_mapping;
                m_p_server_mapping = NULL;
            }

            // if(m_p_server_prediction != NULL) {
            //     delete m_p_server_prediction;
            //     m_p_server_prediction = NULL;
            // }

            if(m_p_server_perception_2 != NULL) {
                delete m_p_server_perception_2;
                m_p_server_perception_2 = NULL;
            }

            if(m_p_head_yaw_tracker != NULL) {
                delete m_p_head_yaw_tracker;
                m_p_head_yaw_tracker = NULL;
            }

            if(m_p_head_pitch_tracker != NULL) {
                delete m_p_head_pitch_tracker;
                m_p_head_pitch_tracker = NULL;
            }

            if(m_p_vehicle_tracker != NULL) {
                delete m_p_vehicle_tracker;
                m_p_vehicle_tracker = NULL;
            }

            // depth
            if (m_p_depth_processor){
                delete(m_p_depth_processor);
            }

            delete[] bounding_box;
            delete[] control_cmd;
        }

        bool AlgoADP::init()
        {
            //mRawDataInterface->ExecuteHeadMode(0);
            //mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);

            raw_depth.image = cv::Mat::zeros(cv::Size(320, 240), CV_16UC1);
            raw_depth.timestampSys = 0;
            t_old = 0;
            imgstream_en = true;
            nStep = 0;

            mRawDataInterface->getMaincamParam(raw_camerapara);
            mRawDataInterface->getCalibrationDS4T(calib);

            string serial = RawData::retrieveRobotSerialNumber();
            //ALOGD("robot serial: %s", serial.c_str());
            //ALOGD("robot model: %d", RawData::retrieveRobotModel());
            m_is_init_succed = true;

            m_p_server_control = new SocketServer(8080);
            m_p_server_perception = new SocketServer(8081);
            m_p_server_estimation = new SocketServer(8082);
            m_p_server_mapping = new SocketServer(8083);
            //m_p_server_prediction = new SocketServer(8084);
            m_p_server_perception_2 = new SocketServer(8085);

            m_timestamp_start = mRawDataInterface->getCurrentTimestampSys();

            initLocalMapping();

            float head_kp, head_ki, head_kd;
            float vehicle_kp, vehicle_ki, vehicle_kd;
            loadConfig(head_kp, head_ki, head_kd, vehicle_kp, vehicle_ki, vehicle_kd);

            m_p_head_yaw_tracker = new PID(0.1, 0.5, -0.5, head_kp,  head_ki, head_kd);
            m_p_head_pitch_tracker = new PID(0.1, 0.1, -0.1, 0.1, 0.01, 0.01);
            m_p_vehicle_tracker = new PID(0.1, 1.0, -0.5, vehicle_kp, vehicle_ki, vehicle_kd);

            m_direction_head_test = 0;

            // depth
            m_p_depth_processor = new DepthPreprocess();

            // detection
            m_is_detected = false;
            bounding_box = new float[5];
            control_cmd = new float[2];

            return m_is_init_succed;
        }

        void AlgoADP::UpdateAccel(MTPoint val)
        {
            raw_accel = val;
        }

        void AlgoADP::UpdateGyro(MTPoint val)
        {
            raw_gyro = val;
        }

        void AlgoADP::toggleImgStream()
        {
            imgstream_en = !imgstream_en;
        }

        void AlgoADP::setVLSopen(bool en)
        {
            if(en)
                mRawDataInterface->startVLS(false);
            else
                mRawDataInterface->stopVLS();
        }

        void AlgoADP::startPoseRecord(std::string save_folder, int64_t save_time)
        {
            pose_isRecording = true;
            nStep = 0;
            m_folder_socket = "/sdcard/socket/";
            createFolder(m_folder_socket);
            m_state_file.open(m_folder_socket + "state.txt");
        }

        void AlgoADP::stopPoseRecord()
        {
            pose_isRecording = false;
            m_state_file.close();
        }

        bool AlgoADP::step()
        {
            ALOGD("step start");
            /*! Get start timestamp for calculating algorithm runtime */
            auto start = std::chrono::high_resolution_clock::now();

            if(!m_is_init_succed){
                //ALOGD("AlgoADP: init false");
                return false;
            }

            if(imgstream_en){
                mRawDataInterface->retrieveDepth(raw_depth, false);
                if(raw_depth.timestampSys==0)
                {
                    //ALOGD("depth wrong");
                    return false;
                }
                // check if depth is duplicate
                if(t_old==raw_depth.timestampSys)
                {
                    //ALOGD("depth duplicate: %lld",raw_depth.timestampSys/1000);
                    return true;
                }
                else
                {
                    t_old=raw_depth.timestampSys;
                    //ALOGD("depth correct");
                }
            }

            /*! **********************************************************************
             * **** Local
             * ********************************************************************* */
            mRawDataInterface->retrieveOdometry(raw_odometry, -1);
            prepare_localmap_and_pose_for_controller_g1();

            /*! **********************************************************************
             * **** Cloud
             * ********************************************************************* */
            this->stepServer();

            /*! **********************************************************************
             * **** Processing the algorithm with all input and sensor data **********
             * ********************************************************************* */
            string contents;

            ALOGD("rendering start");
            auto start_render = std::chrono::high_resolution_clock::now();
            if(m_isRender)
            {
                /*! Copy internal canvas to intermediate buffer mDisplayIm */
                renderDisplay();
                setDisplayData();
            }

            auto end_render = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_render = end_render-start_render;
            ALOGD("render time: %f",elapsed_render.count());

            /*! Calculate the algorithm runtime */
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end-start;
            ALOGD("step time: %f",elapsed.count());
            {
                std::lock_guard<std::mutex> lock(mMutexTimer);
                m_ptime = elapsed.count()*0.5 + m_ptime*0.5;
            }

            return true;
        }

        void AlgoADP::ImgProcessing(){
            ALOGD("in ImgProcessing");
            this->sendImage();
            this->receiveBbox();
            this->sendPositions();
        }

        void AlgoADP::stepServer() {
            ALOGD("in step Server");

            this->sendStates();

            this->ImgProcessing();

            this->receiveCtrl();


            //std::thread t1(&AlgoADP::ImgProcessing, this);
            //std::thread t1(&AlgoADP::receiveCtrl, this);
            //t1.join();
            //std::vector<std::thread> threads;
            //threads.push_back(std::thread(&AlgoADP::ImgProcessing, this));
            //threads.push_back(std::thread(&AlgoADP::sendStates, this));
            //threads.push_back(std::thread(&AlgoADP::receiveCtrl, this));
            //this->ImgProcessing();
            //std::for_each(threads.begin(),threads.end(),std::mem_fn(&std::thread::join));
        }

        

        void AlgoADP::sendImage(){
            ALOGD("in sendImage");
            auto startimg = std::chrono::high_resolution_clock::now();
            //Get the image from Camera
            //std::lock_guard<std::mutex> guard(mMutexImg);

            mRawDataInterface->retrieveColor(raw_color, true);

            // ALOGD("raw_color: (%lld,%d,%d,%d)", raw_color.timestampSys, raw_color.image.cols, raw_color.image.rows, raw_color.image.channels());
            //Handle Exceptions when Camera is not accessible or Connection lost to server
            if (raw_color.image.empty()) {
                ALOGW("empty raw_color image");
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys());
                auto endimg = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsedimg = endimg-startimg;
                ALOGD("img time: %f",elapsedimg.count());
                return;
            }
            if (m_p_server_control->isStopped()) {
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys());
                ALOGW("server stopped");
                auto endimg = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsedimg = endimg-startimg;
                ALOGD("img time: %f",elapsedimg.count());
                return;
            }
            if (!m_p_server_control->isConnected()) {
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys());
                ALOGW("server disconnected");
                mRawDataInterface->ExecuteHeadMode(0);
                mRawDataInterface->ExecuteHeadPos(head_yaw, head_pitch , 0);
                auto endimg = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsedimg = endimg-startimg;
                ALOGD("img time: %f",elapsedimg.count());
                return;
            }


            //send image
            cv::Mat image_send;
            cv::resize(raw_color.image, image_send, cv::Size(640/m_down_scale, 480/m_down_scale));
            int info_send_image = m_p_server_perception->sendImage(image_send, 640/m_down_scale, 480/m_down_scale);
            if (info_send_image < 0) {
                ALOGW("server send image failed");
                mRawDataInterface->ExecuteHeadMode(0);
                mRawDataInterface->ExecuteHeadPos(head_yaw, head_pitch, 0);
                //return;
            }
            else {
                ALOGW("server send image succeeded");
            }

            auto endimg = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedimg = endimg-startimg;
            ALOGD("img time: %f",elapsedimg.count());




        }

        void AlgoADP::sendStates() {
            ALOGD("in sendStates");
            // Send the state of the Loomo {x,y,heading,vx,w,ax,ay}
            auto startstate = std::chrono::high_resolution_clock::now();
            float* states = new float[5];
            mRawDataInterface->retrieveOdometry(raw_odometry, -1);
            states[0] = float(raw_odometry.twist.pose.x);
            states[1] = float(raw_odometry.twist.pose.y);
            states[2] = float(raw_odometry.twist.pose.orientation);
            states[3] = raw_odometry.twist.velocity.linear_velocity;
            states[4] = raw_odometry.twist.velocity.angular_velocity;

            int info_send_state = m_p_server_estimation->sendFloats(states, 5);

            delete[] states;
            auto endstates = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedstates = endstates-startstate;
            ALOGD("states time: %f",elapsedstates.count());
        }

        void AlgoADP::receiveBbox(){
            ALOGD("in receive bbox");
            // Receive Bounding Box coordinates (5 floats)
            auto startbbox = std::chrono::high_resolution_clock::now();
            float* floats_recv = new float[25];
            int rcv_info_test = m_p_server_perception_2->recvFloats(floats_recv, 25);
            if (rcv_info_test < 0) {
                ALOGD("server rcv float failed");
                return;
            }
            // else {
            //     for (int i = 0; i < 5; i++)
            //     {
            //         bounding_box[i] = *(float*)&floats_recv[i];
            //         ALOGD("server bounding_box #%d: %.2f", i, bounding_box[i]);
            //     }
            // }
            bounding_box = floats_recv;
            delete[] floats_recv;
            auto endbbox = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedbbox = endbbox-startbbox;
            ALOGD("bbox time: %f",elapsedbbox.count());

        }

        void AlgoADP::sendPositions(){
            ALOGD("in sendPosition");
            auto startpos = std::chrono::high_resolution_clock::now();
            m_roi_color.width = int(bounding_box[2] * m_down_scale);
            m_roi_color.height = int(bounding_box[3] * m_down_scale);
            m_roi_color.x = (bounding_box[0] - 320/m_down_scale) * m_down_scale + 320;
            m_roi_color.y = (bounding_box[1] - 240/m_down_scale) * m_down_scale + 240;
            // convert from center to top-left corner
            // m_roi_color.x = int(m_roi_color.x - m_roi_color.width/2);
            // m_roi_color.y = int(m_roi_color.y - m_roi_color.height/2);


            if (bounding_box[4] > 0.5)
                m_is_detected = true;
            else
                m_is_detected = false;


            float target_theta_wrt_head =0.0f;
            if (m_is_detected) {
                ExtractTarget(m_target_distance, target_theta_wrt_head);
                m_target_theta = target_theta_wrt_head + raw_headpos.yaw;
            }
            else {
                m_target_distance = 0.0f;
                m_target_theta = 0.0f;
                target_theta_wrt_head = 0.0f;
            }

            //Once we extracted the depth of the target can move head
            auto start_head = std::chrono::high_resolution_clock::now();


            this->trackHead(m_target_theta);


            auto end_head = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_head = end_head-start_head;
            ALOGD("head time: %f",elapsed_head.count());
            // send positions: distance to prediction and mapping

            float* position_obj = new float[10];
            position_obj[0] = m_target_distance * cos(m_target_theta);
            position_obj[1] = m_target_distance * sin(m_target_theta);

            //Temporary locked to only 1 Object being detected
            for (int i=2; i<10; i++){
                position_obj[i]=0.0;
            }
            //int info_send_perception = m_p_server_prediction->sendFloats(position_obj, 10);
            int info_send_mapping = m_p_server_mapping->sendFloats(position_obj, 10);
            delete[] position_obj;
            auto enddepth = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapseddepth = enddepth-startpos;
            ALOGD("depth time: %f",elapseddepth.count());

        }

        void AlgoADP::receiveCtrl(){
            ALOGD("in recvCtrl");
            auto startctrl = std::chrono::high_resolution_clock::now();
            float* floats_recv_control = new float[2];
            int rcv_info_test_control = m_p_server_control->recvFloats(floats_recv_control, 2);
            ccmd = floats_recv_control;
            delete[] floats_recv_control;
            control_cmd[0] = 0.0;
            control_cmd[1] = 0.0;

            auto rcv_ctrl = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedrcv = rcv_ctrl-startctrl;
            ALOGD("rcv command time: %f",elapsedrcv.count());

            if (rcv_info_test_control < 0)
            {
                ALOGD("server rcv float failed");
                this->trackVehicle(0.0, 0.0);
                return;
            }

            else
            {
                control_cmd[0] = *(float*)&ccmd[0];
                control_cmd[1] = *(float*)&ccmd[1];
            }

            // send control cmds
            this->trackVehicle(control_cmd[0], control_cmd[1]);
            auto track = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedtrack = track-rcv_ctrl;
            ALOGD("track vehichle time: %f",elapsedtrack.count());
            // ALOGD("Vehicle Target: target_distance = %f, target_theta_wrt_head = %f", m_target_distance, target_theta_wrt_head);
            auto endctrl = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsedctrl = endctrl-startctrl;
            ALOGD("ctrl time: %f",elapsedctrl.count());
        }

        void AlgoADP::switchHeadTracker() {
            m_is_track_head = !m_is_track_head;
        }

        void AlgoADP::switchVehicleTracker() {
            m_is_track_vehicle = !m_is_track_vehicle;
            m_is_track_head = m_is_track_vehicle;
        }

        void AlgoADP::safeControl(float v, float w) {
            if (m_safety_control) {
                const float kCloseObstacleThres = 1.0;
                if(m_ultrasonic_average < kCloseObstacleThres*1000){
                    if (v > 0 && std::abs(v/std::fmax(0.01,w)) > 0.8){
                        StampedVelocity velocity;
                        mRawDataInterface->retrieveBaseVelocity(velocity);
                        float v_emergency = std::min(0.0, 0.2-velocity.vel.linear_velocity);
                        ALOGE("command dangerous: (%f,%f), current vel: (%f,%f), ultrasonic_average = %f: v_emergency = %f", v, w, velocity.vel.linear_velocity, velocity.vel.angular_velocity, m_ultrasonic_average, v_emergency);
                        mRawDataInterface->ExecuteCmd(v_emergency, 0.0f, 0);
                        return;
                    }
                }
            }

            mRawDataInterface->ExecuteCmd(v, w, 0);
            ALOGD("command safe: (%f,%f)", v, w);
        }

        float AlgoADP::runTime()
        {
            std::lock_guard<std::mutex> lock(mMutexTimer);
            ALOGD("in Runtime, m_ptime value: %f",m_ptime);
            return m_ptime;
        }

        bool AlgoADP::showScreen(void* pixels) // canvas 640x360, RGBA format
        {
            {
                std::lock_guard<std::mutex> lock(mMutexDisplay);
                if(mDisplayIm.empty())
                    return false;
                cv::cvtColor(mDisplayIm, mDisplayData, CV_BGR2RGBA);
            }
            memcpy(pixels, (void *)mDisplayData.data, mDisplayData.cols * mDisplayData.rows * 4);
            return true;
        }

        void AlgoADP::setDisplayData()
        {
            std::lock_guard<std::mutex> lock(mMutexDisplay);
            mDisplayIm = canvas.clone();
        }

        void AlgoADP::createFolder(std::string new_folder_name)
        {
            std::string cmd_str_rm = "rm -rf \"" + new_folder_name + "\"";
            system(cmd_str_rm.c_str());
            ALOGD("Command %s was executed. ", cmd_str_rm.c_str());
            std::string cmd_str_mk = "mkdir \"" + new_folder_name + "\"";
            system(cmd_str_mk.c_str());
            ALOGD("Command %s was executed. ", cmd_str_mk.c_str());
        }

        std::string AlgoADP::getDebugString()
        {
            std::string str;

            if (m_safety_control){
                str = "，safety ON";
            }
            else {
                str = "，safety OFF";
            }

            int raw_errorcode = mRawDataInterface->retrieveDeviceErrorCode();
            if (raw_errorcode != 0)
                str += ", err:"+ToString(raw_errorcode);

            return str;
        }

        bool AlgoADP::initLocalMapping()
        {
            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }
            float mapsize = 8.0;
            float m_map_resolution = 0.05;
            m_p_local_mapping = new ninebot_algo::local_mapping::LocalMapping(mapsize, m_map_resolution);
            if(m_p_local_mapping == NULL)
                return false;
            CalibrationInfoDS4T calib;
            mRawDataInterface->getCalibrationDS4T(calib);
            float fx = calib.depth.focalLengthX;
            float fy = calib.depth.focalLengthY;
            float px = calib.depth.pricipalPointX;
            float py = calib.depth.pricipalPointY;
            m_p_local_mapping->setLidarRange(5.0);
            m_p_local_mapping->setLidarMapParams(0.6, true);
            m_p_local_mapping->setDepthCameraParams(px, py, fx, fy, 1000);
            m_p_local_mapping->setDepthRange(3.5, 0.35, 0.9, 0.1);
            m_p_local_mapping->setDepthMapParams(1.0, 10, false, -1);
            m_p_local_mapping->setUltrasonicRange(0.5);
            m_map_width =  mapsize / m_map_resolution;
            m_map_height = mapsize / m_map_resolution;

            return true;
        }

        bool AlgoADP::prepare_localmap_and_pose_for_controller_g1() {
            // generate mapping
            mRawDataInterface->retrieveDepth(raw_depth, true);

            if(raw_depth.timestampSys == 0) {
                ALOGD("localmap: depth wrong");
                return false;
            }
            StampedMat local_depth(raw_depth.image, raw_depth.timestampSys);

            // merge all TF requests
            ninebot_tf::vector_req_t reqList;
            ninebot_tf::vector_tf_msg_t resList;
            reqList.push_back(ninebot_tf::tf_request_message("base_center_ground_frame", "world_odom_frame", raw_depth.timestampSys, 500));
            reqList.push_back(ninebot_tf::tf_request_message("rsdepth_center_neck_fix_frame", "world_odom_frame", raw_depth.timestampSys, 500));
            mRawDataInterface->getMassiveTfData(&reqList,&resList);
            if(resList.size()!=2){
                ALOGE("getMassiveTfData wrong");
                return false;
            }

            // clear history
            m_p_local_mapping->clearMap();
            // m_p_local_mapping->setMapValue(raw_odometry.twist.pose.x, raw_odometry.twist.pose.y, 254, 1.5);

            ninebot_tf::tf_message tf_msg = resList[0];
            ninebot_tf::tf_message tf_msg2 = resList[1];
            if (tf_msg.err==0 && tf_msg2.err==0) {
                StampedPose odom_pos = tfmsgTo2DPose(tf_msg);
                Eigen::Isometry3f depth_pose = PoseToCamcoor(tfmsgToPose(tf_msg2));
                // process depth
                m_p_local_mapping->processDepthFrame(local_depth, odom_pos, depth_pose);
            }
            else {
                ALOGE("localmap: Could not find tf pose with specified depth ts, error code: %d, %d", tf_msg.err, tf_msg2.err);
            }

            // StampedFloat ultrasonic;
            mRawDataInterface->retrieveUltrasonic(raw_ultrasonic);
            m_ultrasonic_buffer.push_back(raw_ultrasonic.value);
            if (m_ultrasonic_buffer.size() > 3) {
                m_ultrasonic_buffer.pop_front();
            }
            float ultrasonic_sum = 0;
            for (auto ultrasonic_element : m_ultrasonic_buffer) {
                ultrasonic_sum += ultrasonic_element;
            }
            m_ultrasonic_average = ultrasonic_sum / m_ultrasonic_buffer.size();

            m_p_local_mapping->getDepthMapWithFrontUltrasonic(m_local_map, false, tfmsgTo2DPose(resList[0]), m_ultrasonic_average);

            return true;
        }

        void AlgoADP::trackHead(const float target_theta_head) {
            auto start_headtrack = std::chrono::high_resolution_clock::now();
            if(m_is_track_head) {
                float w = 0.0f;
                float head_pitch_speed = 0.0f;
                float head_yaw_speed = 0.0f;
                mRawDataInterface->retrieveHeadPos(raw_headpos);

                // yaw
                if (m_p_head_yaw_tracker) {
                    if (m_is_detected)
                        head_yaw_speed = m_p_head_yaw_tracker->calculate(target_theta_head,
                                                                         raw_headpos.yaw);
                    else
                        head_yaw_speed = m_p_head_yaw_tracker->calculate(raw_headpos.yaw,
                                                                         raw_headpos.yaw);
                }

                // pitch
                if (m_p_head_pitch_tracker) {
                    head_pitch_speed = m_p_head_pitch_tracker->calculate(head_pitch, raw_headpos.pitch);
                }
                //const int HEIGHT_COLOR_IMG = 480;
                //if (m_roi_color.y < HEIGHT_COLOR_IMG / 6) {
                //    if (raw_headpos.pitch > 1.0f) {
                //        head_pitch_speed = -0.1f;
                //    } else {
                //        head_pitch_speed = 0.3f;
                //    }
                //} else if (m_roi_color.y > (HEIGHT_COLOR_IMG / 4)) {
                //    if (raw_headpos.pitch < 0.05f) {
                //        head_pitch_speed = 0.1f;
                //    } else {
                //        head_pitch_speed = -0.3f;
                //    }
                //} else {
                //    head_pitch_speed = 0.0f;
                //}

                mRawDataInterface->ExecuteHeadMode(1);
                if (m_is_track_head)
                    mRawDataInterface->ExecuteHeadSpeed(head_yaw_speed, head_pitch_speed, 0);
                else
                    mRawDataInterface->ExecuteHeadSpeed(0.0f, head_pitch_speed, 0);
                ALOGD("trackHead: target_theta_head = %f, yaw_speed = %f, pitch_speed = %f", target_theta_head, head_yaw_speed, head_pitch_speed);
            }

            auto end_headtrack = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_headtrack = end_headtrack-start_headtrack;
            ALOGD("head speed time: %f",elapsed_headtrack.count());

        }

        void AlgoADP::trackVehicle(const float vehicle_linear_velocity, const float vehicle_angular_velocity) {
            //float vehicle_linear_velocity, vehicle_angular_velocity;
            if (m_is_track_vehicle) {
                // if (m_is_detected && target_distance > 0) {
                //     if (target_distance > 1.5) {
                //         vehicle_linear_velocity = m_p_vehicle_tracker->calculate(target_distance - 1.2f, 0.0);
                //         vehicle_angular_velocity = vehicle_linear_velocity / fmin(2.0f,target_distance) * target_theta * 2.0f;
                //     }
                //     else {
                //         vehicle_linear_velocity = m_p_vehicle_tracker->calculate(0.0, 0.0);
                //         vehicle_linear_velocity = 0.0f;
                //         if (abs(target_theta) > 0.2) {
                //             vehicle_angular_velocity = target_theta * 1.0f;
                //         }
                //         else {
                //             vehicle_angular_velocity = 0.0f;
                //         }
                //     }
                // }
                // else {
                //     vehicle_linear_velocity = m_p_vehicle_tracker->calculate(0.0, 0.0);
                //     vehicle_linear_velocity = 0.0f;
                //     vehicle_angular_velocity = 0.0f;
                // }

                this->safeControl(vehicle_linear_velocity, vehicle_angular_velocity);
            }
            ALOGD("trackVehicle: vehicle_linear_velocity = %.2f, vehicle_angular_velocity=%.2f", vehicle_linear_velocity, vehicle_angular_velocity);
        }

        void AlgoADP::ExtractTarget(float & target_distance, float & target_theta_wrt_head){
            mRawDataInterface->retrieveHeadPos(raw_headpos);
            m_p_depth_processor->setPitch(raw_headpos.pitch);
            ALOGD("ExtractTarget: setPitch = %f", raw_headpos.pitch);

            cv::Mat hist_image;
            bool is_dist_valid = m_p_depth_processor->process(raw_depth, m_roi_color, m_roi_depth, hist_image, target_distance, target_theta_wrt_head);
            // target_theta_wrt_head = -target_theta_wrt_head;

            ALOGD("ExtractTarget: is_dist_valid = %d", is_dist_valid);
            ALOGD("ExtractTarget: target_distance = %f", target_distance);
            ALOGD("ExtractTarget: raw_headpos.yaw = %.2f, target_theta_wrt_head = %.2f, target_theta_wrt_head (manual) = %.2f", raw_headpos.yaw, target_theta_wrt_head, (float) 0.5f - (m_roi_color.x + m_roi_color.width / 2.0f ) / 640.0f );

            // target_theta_wrt_head = (float) m_roi_color.x / 640.f - 0.5f;
        }

        void AlgoADP::renderDisplay() {
            /*! Draw the result to canvas */
            canvas.setTo(240);
            float size_color, size_depth;
            if (render_depth){
                size_color=0.5;
                size_depth=1.0;
            }
            else{
                size_color=0.75;
                size_depth=0.0;
            }
            if(imgstream_en){
                if(!raw_color.image.empty()){
                    // std::lock_guard<std::mutex> guard(mMutexImg);
                    cv::Mat show_color;
                    const float resize_show_color = size_color;
                    cv::resize(raw_color.image, show_color, cv::Size(), resize_show_color, resize_show_color);
                    if (m_is_detected)
                        cv::rectangle(show_color, cv::Rect(int(m_roi_color.x * resize_show_color), int(m_roi_color.y * resize_show_color), int(m_roi_color.width * resize_show_color), int(m_roi_color.height * resize_show_color)), Scalar(0, 0, 0), 10);
                    cv::Mat flip_color;               // dst must be a different Mat
                    cv::flip(show_color, flip_color, 1);     // because you can't flip in-place (leads to segfault)
                    cv::Mat ca1 = canvas(cv::Rect(0, 0, flip_color.cols, flip_color.rows));
                    flip_color.copyTo(ca1);
                }
                if(!raw_depth.image.empty() && render_depth){
                    cv::Mat show_depth, rescale_depth;
                    const float resize_show_depth = size_depth;
                    cv::resize(raw_depth.image, rescale_depth, cv::Size(), resize_show_depth, resize_show_depth);
                    rescale_depth /= 10;
                    rescale_depth.convertTo(show_depth, CV_8U);
                    applyColorMap(show_depth, show_depth, cv::COLORMAP_JET);
                    if (m_is_detected)
                        cv::rectangle(show_depth, cv::Rect(int(m_roi_depth.x * resize_show_depth), int(m_roi_depth.y * resize_show_depth), int(m_roi_depth.width * resize_show_depth), int(m_roi_depth.height * resize_show_depth)), Scalar(0, 0, 0), 10);
                    ALOGD("m_roi_depth: (%f,%f,%f,%f)", m_roi_depth.x, m_roi_depth.y, m_roi_depth.width, m_roi_depth.height);
                    cv::Mat flip_depth;                         // dst must be a different Mat
                    cv::flip(show_depth, flip_depth, 1);        // because you can't flip in-place (leads to segfault)
                    cv::Mat ca2 = canvas(cv::Rect(320, 0, flip_depth.cols, flip_depth.rows));
                    flip_depth.copyTo(ca2);
                }

                string contents;
                if (m_is_track_head){
                    contents = "Head: On";
                }
                else {
                    contents = "Head: Off";
                }
                putText(canvas, contents, cv::Point(0, 300), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);

                if (m_is_track_vehicle){
                    contents = "Wheel: On";
                }
                else {
                    contents = "Wheel: Off";
                }
                putText(canvas, contents, cv::Point(0, 330), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);

                if (m_target_distance > 0) {
                    contents = "Depth: " + ToString(m_target_distance);
                    putText(canvas, contents, cv::Point(text_pos, 300), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                    contents = "Angle: " + ToString(m_target_theta/3.14*180.0);
                    putText(canvas, contents, cv::Point(text_pos, 330), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                }
                contents = "V: " + ToString(control_cmd[0]);
                putText(canvas, contents, cv::Point(text_pos-200, 300), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                contents = "W: " + ToString(control_cmd[1]);
                putText(canvas, contents, cv::Point(text_pos-200, 330), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
            }
        }

        bool AlgoADP::loadConfig(float & head_kp, float & head_ki, float & head_kd, float & vehicle_kp, float & vehicle_ki, float & vehicle_kd)
        {
            head_kp = 0.0f;
            head_ki = 0.0f;
            head_kd = 0.0f;
            vehicle_kp = 0.50f;
            vehicle_ki = 0.01f;
            vehicle_kd = 0.01f;
            m_down_scale = 8;
            PARAM_SETTING param_setting = (PARAM_SETTING)0;
            std::ifstream ifconfig;
            std::string line;
            std::string filename = "/sdcard/follow.cfg";
            ifconfig.open(filename);
            if (!ifconfig.is_open())
            {
                ALOGD("failed to open config file '%s'", filename.c_str());
                return false;
            }

            ALOGD("opened config file '%s'", filename.c_str());

            int idx_config = 0;
            while (std::getline(ifconfig, line)) {
                std::istringstream iss(line);
                std::string parameter;
                while (iss >> parameter)
                {
                    char c = parameter[0];
                    if (c == '#')
                        break;

                    const char *_Ptr = parameter.c_str();
                    char *_Eptr;
                    float param_value = strtod(_Ptr, &_Eptr);

                    switch (param_setting)
                    {
                        case HEAD_KP:
                            ALOGD("config loaded, HEAD_KP = %f", param_value);
                            head_kp = param_value;
                            break;
                        case HEAD_KI:
                            ALOGD("config loaded, HEAD_KI = %f", param_value);
                            head_ki = param_value;
                            break;
                        case HEAD_KD:
                            ALOGD("config loaded, HEAD_KD = %f", param_value);
                            head_kd = param_value;
                            break;
                        case VEHICLE_KP:
                            ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                            vehicle_kp = param_value;
                            break;
                        case VEHICLE_KI:
                            ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                            vehicle_ki = param_value;
                            break;
                        case VEHICLE_KD:
                            ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                            vehicle_kd = param_value;
                            break;
                        case IMG_DOWNSCALE:
                            ALOGD("config loaded, m_down_scale = %f", param_value);
                            m_down_scale = int(param_value);
                            break;
                    }
                }
                idx_config++;
                param_setting = (PARAM_SETTING)idx_config;
            }

            ALOGD("config: head_kp = %.2f, head_ki = %.2f, head_kd = %.2f", head_kp, head_ki, head_kd);

            return true;
        }

    } // namespace follow_algo
} // namespace ninebot_algo
