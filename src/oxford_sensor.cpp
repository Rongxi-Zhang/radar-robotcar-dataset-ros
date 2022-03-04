#include <oxford_sensor.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

namespace sensor
{
    // Names of all frames(public, must be locked)
    const string mono_left_frame = "Grasshopper2_left", mono_right_frame = "Grasshopper2_Righr", mono_rear_frame = "Grasshopper2_Rear";
    const string stereo_frame = "Bumblebee", stereo_centre_frame = "Bumblebee_centre", stereo_right_frame = "Bumblebee_right";
    const string radar_frame = "CTS350-X", lidar_left_frame = "HDL-32E_Left", lidar_right_frame = "HDL-32E_right";
    const string gps_frame = "SPAN-CPT";

    //------------------------------ class CameraCalibration-----------------------------//
    void CameraCalibration::ToCameraInfo(sensor_msgs::CameraInfo &info)
    {
        for (size_t i = 0; i < 5; ++i)
            info.D.push_back(D(0, i)); // unknown, push 0

        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 4; ++j)
            {
                if (j < 3)
                {
                    info.K[i * 3 + j] = K(i, j);
                    info.R[i * 3 + j] = R(i, j);
                }
                info.P[i * 4 + j] = P(i, j);
            }
        info.distortion_model = "plumb_bob";
    }

    void CameraCalibration::readcali(sensor_msgs::CameraInfo &info, string filename)
    {
        ifstream infile(filename, ios::in);
        string strline;
        getline(infile, strline);

        size_t index;

        index = strline.find_first_of(' ');
        K(0, 0) = stod(strline.substr(0, index));
        strline = strline.substr(++index);

        index = strline.find_first_of(' ');
        K(1, 1) = stod(strline.substr(0, index));
        strline = strline.substr(++index);

        index = strline.find_first_of(' ');
        K(0, 2) = stod(strline.substr(0, index));
        strline = strline.substr(++index);

        K(1, 2) = stod(strline);

        R << 0.0, -0.0, 1.0, 1.0, 0.0, -0.0, 0.0, 1.0, 0.0;

        P.block(0, 0, 3, 3) = R;
        getline(infile, strline);
        index = strline.find_last_of(' ');
        P(0, 3) = stod(strline.substr(index, strline.size() - index));
        getline(infile, strline);
        index = strline.find_last_of(' ');
        P(1, 3) = stod(strline.substr(index, strline.size() - index));

        infile.close();

        ToCameraInfo(info);
    }
    //--------------------------------class mono------------------------------------//
    mono::mono(const string &config_file, const string &dataset_path, ros::NodeHandle node)
        : n(node)
    {
        mono_model = config_file + "/models/";
        mono_path = dataset_path;

        ifstream leftfile, rightfile, rearfile;

        leftfile.open(dataset_path + "/mono_left.timestamps", ios::in);
        if (leftfile.is_open())
        {
            cout << "Reading monoleft timestamps from this txt." << std::endl;
            string strline, subline;
            while (getline(leftfile, strline))
            {
                subline = strline.substr(0, strline.find_last_of(' '));
                LeftTimeList.push_back(subline);
            }
            leftfile.close();
        }
        else
            cout << "Unkown mono_left timestamps from this txt." << std::endl;

        rightfile.open(dataset_path + "/mono_right.timestamps", ios::in);
        if (rightfile.is_open())
        {
            cout << "Reading mono_right timestamps from this txt." << std::endl;
            string strline, subline;
            while (getline(rightfile, strline))
            {
                subline = strline.substr(0, strline.find_last_of(' '));
                RightTimeList.push_back(subline);
            }
            rightfile.close();
        }
        else
            cout << "Unkown mono_right timestamps from this txt." << std::endl;

        rearfile.open(dataset_path + "/mono_rear.timestamps", ios::in);
        if (rearfile.is_open())
        {
            cout << "Reading mono_rear timestamps from this txt." << std::endl;
            string strline, subline;
            while (getline(rearfile, strline))
            {
                subline = strline.substr(0, strline.find_last_of(' '));
                RearTimeList.push_back(subline);
            }
            rearfile.close();
        }
        else
            cout << "Unkown mono_rear timestamps from this txt." << std::endl;

        CameraCalibration mono_left_cali, mono_right_cali, mono_rear_cali;
        string mono_intrinsics = mono_model + "mono_left.txt";
        mono_left_cali.readcali(info_mono_left, mono_intrinsics);
        mono_intrinsics = mono_model + "mono_right.txt";
        mono_right_cali.readcali(info_mono_right, mono_intrinsics);
        mono_intrinsics = mono_model + "mono_rear.txt";
        mono_rear_cali.readcali(info_mono_rear, mono_intrinsics);

        info_mono_left.height = info_mono_left.width = 1024;
        info_mono_right.height = info_mono_right.width = 1024;
        info_mono_rear.height = info_mono_rear.width = 1024;
    }

    void mono::publish(int flag)
    {
        // From SDK extrinsics
        vector<float> trans_left = {-0.0905, 1.6375, 0.2803, 0.2079, -0.2339, 1.2321};
        vector<float> trans_right = {-0.2587, -1.6810, 0.3226, -0.1961, -0.2469, -1.2675};
        vector<float> trans_rear = {-2.0582, 0.0894, 0.3675, -0.0119, -0.2498, 3.1283};

        ros::Rate loop_rate(17);
        cout << "Publishing mono images..." << std::endl;

        ros::Publisher pubImage, pubCameraInfo;
        sensor_msgs::ImagePtr Image_Msg;

        switch (flag)
        {
        case 0:
            pubImage = n.advertise<sensor_msgs::Image>("oxford/mono/left", 10);
            pubCameraInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/mono/info/left", 10);
            for (size_t i = 0; i < LeftTimeList.size(); ++i)
            {
                string name = LeftTimeList[i] + ".png";
                string path = mono_path + "/mono_left/" + name;
                cv::Mat image = cv::imread(path, -1);
                cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);

                Image_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                Image_Msg->header.frame_id = mono_left_frame;
                Image_Msg->header.stamp = sensor::Timestamp2Rostime(LeftTimeList[i]);

                info_mono_left.header.frame_id = mono_left_frame;
                info_mono_left.header.stamp = Image_Msg->header.stamp;

                if (!save_flag)
                {
                    pubImage.publish(Image_Msg);
                    pubCameraInfo.publish(info_mono_left);
                    TransformBroadcaster(trans_left, Image_Msg->header.stamp, stereo_frame, mono_left_frame);

                    ros::spinOnce();
                    loop_rate.sleep();
                    if (i < LeftTimeList.size() - 1)
                        loop_rate = sensor::Frequency(LeftTimeList[i + 1], LeftTimeList[i]);
                }
                else
                {
                    oxford_bag.write("oxford/mono/left", Image_Msg->header.stamp, Image_Msg);
                    oxford_bag.write("oxford/mono/info/left", Image_Msg->header.stamp, info_mono_left);
                }
            }

            break;
        case 1:

            pubImage = n.advertise<sensor_msgs::Image>("oxford/mono/right", 10);
            pubCameraInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/mono/info/right", 10);
            for (size_t i = 0; i < RightTimeList.size(); ++i)
            {
                string name = RightTimeList[i] + ".png";
                string path = mono_path + "/mono_right/" + name;
                cv::Mat image = cv::imread(path, -1);
                cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);

                Image_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                Image_Msg->header.frame_id = mono_right_frame;
                Image_Msg->header.stamp = sensor::Timestamp2Rostime(RightTimeList[i]);

                info_mono_right.header.frame_id = mono_right_frame;
                info_mono_right.header.stamp = Image_Msg->header.stamp;

                if (!save_flag)
                {
                    pubImage.publish(Image_Msg);
                    pubCameraInfo.publish(info_mono_right);
                    TransformBroadcaster(trans_right, Image_Msg->header.stamp, stereo_frame, mono_right_frame);

                    ros::spinOnce();
                    loop_rate.sleep();
                    if (i < RightTimeList.size() - 1)
                        loop_rate = sensor::Frequency(RightTimeList[i + 1], RightTimeList[i]);
                }
                else
                {
                    oxford_bag.write("oxford/mono/right", Image_Msg->header.stamp, Image_Msg);
                    oxford_bag.write("oxford/mono/info/right", Image_Msg->header.stamp, info_mono_right);
                }
            }

            break;
        case 2:

            pubImage = n.advertise<sensor_msgs::Image>("oxford/mono/rear", 10);
            pubCameraInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/mono/info/rear", 10);
            for (size_t i = 0; i < RearTimeList.size(); ++i)
            {
                string name = RearTimeList[i] + ".png";
                string path = mono_path + "/mono_rear/" + name;
                cv::Mat image = cv::imread(path, -1);
                cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);

                Image_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                Image_Msg->header.frame_id = mono_rear_frame;
                Image_Msg->header.stamp = sensor::Timestamp2Rostime(RearTimeList[i]);

                info_mono_rear.header.frame_id = mono_rear_frame;
                info_mono_rear.header.stamp = Image_Msg->header.stamp;

                if (!save_flag)
                {
                    pubImage.publish(Image_Msg);
                    pubCameraInfo.publish(info_mono_rear);
                    TransformBroadcaster(trans_rear, Image_Msg->header.stamp, stereo_frame, mono_rear_frame);

                    ros::spinOnce();
                    loop_rate.sleep();
                    if (i < RearTimeList.size() - 1)
                        loop_rate = sensor::Frequency(RearTimeList[i + 1], RearTimeList[i]);
                }
                else
                {
                    oxford_bag.write("oxford/mono/rear", Image_Msg->header.stamp, Image_Msg);
                    oxford_bag.write("oxford/mono/info/rear", Image_Msg->header.stamp, info_mono_rear);
                }
            }

            break;
        default:
            cout << "ERROR MONO FLAG" << endl;
            break;
        }
    }

    //------------------------------ class stereo--------------------------------------//
    stereo::stereo(const string config_file, const string dataset_path, ros::NodeHandle node)
        : n(node)
    {
        stereo_model = config_file + "/models/";
        stereo_path = dataset_path + "/stereo";

        ifstream infile;
        infile.open(dataset_path + "/stereo.timestamps", ios::in);
        if (infile.is_open())
        {
            cout << "Reading stereo timestamps from this txt." << std::endl;
            string strline, subline;
            while (getline(infile, strline))
            {
                subline = strline.substr(0, strline.find_last_of(' '));
                imageTimeList.push_back(subline);
            }
            infile.close();
        }
        else
            cout << "Unkown stereo timestamps from this txt." << std::endl;

        std::cout << "Read stereo models from these txts." << std::endl;
        CameraCalibration wide_left_cali, wide_right_cali;
        CameraCalibration narrow_left_cali, narrow_right_cail;
        // wide model
        string stereo_intrinsics = stereo_model + "stereo_wide_left.txt";
        wide_left_cali.readcali(info_wide_left, stereo_intrinsics);
        stereo_intrinsics = stereo_model + "stereo_wide_right.txt";
        wide_right_cali.readcali(info_wide_right, stereo_intrinsics);
        // narrow model
        stereo_intrinsics = stereo_model + "stereo_narrow_left.txt";
        narrow_left_cali.readcali(info_narrow_left, stereo_intrinsics);
        stereo_intrinsics = stereo_model + "stereo_narrow_right.txt";
        narrow_right_cail.readcali(info_narrow_right, stereo_intrinsics);

        info_wide_left.header.frame_id = stereo_frame;
        info_wide_right.header.frame_id = stereo_right_frame;
        info_narrow_left.header.frame_id = stereo_frame;
        info_narrow_right.header.frame_id = stereo_centre_frame;

        info_narrow_left.height = info_narrow_right.height = info_wide_left.height = info_wide_right.height = 960;
        info_narrow_left.width = info_narrow_right.width = info_wide_left.width = info_wide_right.width = 1280;
    }

    void stereo::publish()
    { // From SDK extinsics
        stereo_mutex.lock();
        vector<float> trans_left = {0, 0, 0, 0, 0, 0};
        vector<float> trans_centre = {0, 0.119997, 0, 0, 0, 0};
        vector<float> trans_right = {0, 0.239983, 0, 0, 0, 0};

        cv::Mat imleft, imcentre, imright;
        string left_path, centre_path, right_path;

        ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("oxford/stereo/Ieft", 10);
        ros::Publisher pubCentreImage = n.advertise<sensor_msgs::Image>("oxford/stereo/centre", 10);
        ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("oxford/stereo/right", 10);

        ros::Publisher pubWideLeftInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/stereo/info/wide_left", 10);
        ros::Publisher pubWideRightInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/stereo/info/wide_right", 10);

        ros::Publisher pubNarrowLeftInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/stereo/info/narrow_left", 10);
        ros::Publisher pubNarrowRightInfo = n.advertise<sensor_msgs::CameraInfo>("oxford/stereo/info/narrow_right", 10);

        ros::Rate loop_rate(16);
        cout << "Publishing stereo images..." << std::endl;
        for (size_t i = 0; i < imageTimeList.size() && ros::ok(); ++i)
        {
            string name = imageTimeList[i] + ".png";
            left_path = stereo_path + "/left/" + name;
            centre_path = stereo_path + "/centre/" + name;
            right_path = stereo_path + "/right/" + name;
            // SDK: BAYER_STEREO = 'gbrg'
            imleft = cv::imread(left_path, -1);
            cv::cvtColor(imleft, imleft, cv::COLOR_BayerGR2BGR);
            sensor_msgs::ImagePtr imLeft_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imleft).toImageMsg();
            imLeft_Msg->header.frame_id = stereo_frame;
            imLeft_Msg->header.stamp = sensor::Timestamp2Rostime(imageTimeList[i]);

            imcentre = cv::imread(centre_path, -1);
            cv::cvtColor(imcentre, imcentre, cv::COLOR_BayerGR2BGR);
            sensor_msgs::ImagePtr imCentre_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imcentre).toImageMsg();
            imCentre_Msg->header.frame_id = stereo_centre_frame;
            imCentre_Msg->header.stamp = imLeft_Msg->header.stamp;

            imright = cv::imread(right_path, -1);
            cv::cvtColor(imright, imright, cv::COLOR_BayerGR2BGR);
            sensor_msgs::ImagePtr imRight_Msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imright).toImageMsg();
            imRight_Msg->header.frame_id = stereo_right_frame;
            imRight_Msg->header.stamp = imLeft_Msg->header.stamp;

            info_wide_left.header.stamp = imLeft_Msg->header.stamp;
            info_wide_right.header.stamp = imLeft_Msg->header.stamp;

            info_narrow_left.header.stamp = imLeft_Msg->header.stamp;
            info_narrow_right.header.stamp = imLeft_Msg->header.stamp;

            if (!save_flag)
            {
                pubLeftImage.publish(imLeft_Msg);
                pubCentreImage.publish(imCentre_Msg);
                pubRightImage.publish(imRight_Msg);

                pubWideLeftInfo.publish(info_wide_left);
                pubWideRightInfo.publish(info_wide_right);

                pubNarrowLeftInfo.publish(info_narrow_left);
                pubNarrowRightInfo.publish(info_narrow_right);

                // TransformBroadcaster(trans_left,imLeft_Msg->header.stamp,"/map",imLeft_Msg->header.frame_id);
                TransformBroadcaster(trans_centre, imLeft_Msg->header.stamp, imLeft_Msg->header.frame_id, imCentre_Msg->header.frame_id);
                TransformBroadcaster(trans_right, imLeft_Msg->header.stamp, imLeft_Msg->header.frame_id, imRight_Msg->header.frame_id);

                ros::spinOnce();
                loop_rate.sleep();
                if (i < imageTimeList.size() - 1)
                    loop_rate = sensor::Frequency(imageTimeList[i + 1], imageTimeList[i]);
            }
            else
            {
                oxford_bag.write("oxford/stereo/Ieft", imLeft_Msg->header.stamp, imLeft_Msg);
                // oxford_bag.write("oxford/stereo/centre", imLeft_Msg->header.stamp, imCentre_Msg);
                oxford_bag.write("oxford/stereo/right", imLeft_Msg->header.stamp, imRight_Msg);

                oxford_bag.write("oxford/stereo/info/wide_left", imLeft_Msg->header.stamp, info_wide_left);
                oxford_bag.write("oxford/stereo/info/wide_right", imLeft_Msg->header.stamp, info_wide_right);

                // oxford_bag.write("oxford/stereo/info/narrow_left", imLeft_Msg->header.stamp, info_narrow_left);
                // oxford_bag.write("oxford/stereo/info/narrow_right", imLeft_Msg->header.stamp, info_narrow_right);

                // only once
                SaveTf2Rosbag(imLeft_Msg->header.stamp);
            }
        }
        stereo_mutex.unlock();
    }
    //------------------------------ class radar--------------------------------------//
    radar::radar(const string config_file, const string dataset_path, ros::NodeHandle node)
        : n(node), encoder_size(5600), range_bins(3768), radar_resolution(0.0432)
    {
        radar_path = dataset_path + "/radar/";
        ifstream infile;
        infile.open(dataset_path + "/radar.timestamps", ios::in);
        if (infile.is_open())
            cout << "Reading radar timestamps from this txt." << std::endl;

        string strline, subline;
        while (getline(infile, strline))
        {
            subline = strline.substr(0, strline.find_last_of(' '));
            RadarTimeList.push_back(subline);
        }
        infile.close();
    }

    void radar::load_radar(cv::Mat &polar_data, cv::Mat &fft_data, std::vector<bool> &valid,
                           std::vector<double> &azimuths, std::vector<int64_t> &timestamps)
    {
        int N = polar_data.rows;
        timestamps = std::vector<int64_t>(N, 0);
        azimuths = std::vector<double>(N, 0);
        valid = std::vector<bool>(N, true);
        fft_data = cv::Mat::zeros(N, range_bins, CV_32F);

        for (int i = 0; i < N; ++i)
        {
            uchar *byteArray = polar_data.ptr<uchar>(i);
            timestamps[i] = *((int64_t *)(byteArray));
            azimuths[i] = *((uint16_t *)(byteArray + 8)) * 2 * M_PI / double(encoder_size);
            valid[i] = byteArray[10] == 255;
            for (int j = 42; j < range_bins; j++)
            {
                fft_data.at<float>(i, j) = (float)*(byteArray + 11 + j) / 255.0;
            }
        }
    }

    void radar::radar_polar_to_cartesian(cv::Mat &fft_data, std::vector<bool> &valid, std::vector<double> &azimuths,
                                         std::vector<int64_t> &timestamps, float cart_resolution, int cart_pixel_width,
                                         bool interpolate_crossover, cv::Mat &cart_img, int output_type)
    {

        float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
        if (cart_pixel_width % 2 == 0)
            cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

        cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
        cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

        for (int j = 0; j < map_y.cols; ++j)
        {
            for (int i = 0; i < map_y.rows; ++i)
            {
                map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
            }
        }

        for (int i = 0; i < map_x.rows; ++i)
        {
            for (int j = 0; j < map_x.cols; ++j)
            {
                map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
            }
        }
        cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
        cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

        double azimuth_step = azimuths[1] - azimuths[0];
        for (int i = 0; i < range.rows; ++i)
        {
            for (int j = 0; j < range.cols; ++j)
            {

                float x = map_x.at<float>(i, j);
                float y = map_y.at<float>(i, j);
                float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) / radar_resolution;
                if (r < 0)
                    r = 0;
                range.at<float>(i, j) = r;

                float theta = atan2f(y, x);
                if (theta < 0)
                    theta += 2 * M_PI;
                angle.at<float>(i, j) = (theta - azimuths[0]) / azimuth_step;
            }
        }
        if (interpolate_crossover)
        {
            cv::Mat a0 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
            cv::Mat aN_1 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
            for (int j = 0; j < fft_data.cols; ++j)
            {
                a0.at<float>(0, j) = fft_data.at<float>(0, j);
                aN_1.at<float>(0, j) = fft_data.at<float>(fft_data.rows - 1, j);
            }
            cv::vconcat(aN_1, fft_data, fft_data);
            cv::vconcat(fft_data, a0, fft_data);
            angle = angle + 1;
        }
        cv::remap(fft_data, cart_img, range, angle, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        if (output_type == CV_8UC1)
        {
            double min, max;
            cv::minMaxLoc(cart_img, &min, &max);
            cart_img.convertTo(cart_img, CV_8UC1, 255.0 / max);
        }
    }

    void radar::publish()
    {
        radar_mutex.lock(); // must lock
        // From SDK extinsics
        vector<float> trans = {-0.71813, 0.12, -0.54479, 0, 0.05, 0};

        ros::Publisher pubPolarRadar = n.advertise<sensor_msgs::Image>("oxford/radar/polar", 10);
        ros::Publisher pubCartRadar = n.advertise<sensor_msgs::Image>("oxford/radar/cart", 10);

        cout << "Publishing radar images..." << std::endl;
        ros::Rate loop_rate(4);
        for (size_t i = 0; i < RadarTimeList.size() && ros::ok(); ++i)
        {

            cv::Mat polar_radar, fft_data, cart_radar;
            std::vector<bool> valid;
            std::vector<double> azimuths;
            std::vector<int64_t> timestamps;
            string polar_path;

            string name = RadarTimeList[i] + ".png";
            polar_path = radar_path + name;

            polar_radar = cv::imread(polar_path, CV_LOAD_IMAGE_GRAYSCALE);
            load_radar(polar_radar, fft_data, valid, azimuths, timestamps);
            radar_polar_to_cartesian(fft_data, valid, azimuths, timestamps, 0.2592, 924, true, cart_radar, CV_8UC1);

            sensor_msgs::ImagePtr Polar_Msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", polar_radar).toImageMsg();
            Polar_Msg->header.frame_id = radar_frame;
            Polar_Msg->header.stamp = sensor::Timestamp2Rostime(RadarTimeList[i]);
            sensor_msgs::ImagePtr Cart_Msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cart_radar).toImageMsg();
            Cart_Msg->header.frame_id = radar_frame;
            Cart_Msg->header.stamp = Polar_Msg->header.stamp;
            if (!save_flag)
            {
                pubCartRadar.publish(Cart_Msg);
                pubPolarRadar.publish(Polar_Msg);
                TransformBroadcaster(trans, Polar_Msg->header.stamp, stereo_frame, Polar_Msg->header.frame_id);

                ros::spinOnce();
                loop_rate.sleep();
                if (i < RadarTimeList.size() - 1)
                    loop_rate = sensor::Frequency(RadarTimeList[i + 1], RadarTimeList[i]);
            }
            else
            {
                oxford_bag.write("oxford/radar/cart", Polar_Msg->header.stamp, Cart_Msg);
                oxford_bag.write("oxford/radar/polar", Polar_Msg->header.stamp, Polar_Msg);
            }
        }
        radar_mutex.unlock(); // unlock
    }
    //------------------------------ class lidar--------------------------------------//
    lidar::lidar(const string config_file, const string dataset_path, ros::NodeHandle node)
        : n(node)
    {
        left_path = dataset_path + "/velodyne_left/";
        right_path = dataset_path + "/velodyne_right/";

        ifstream infile_left, infile_right;
        string strline, subline;

        infile_left.open(dataset_path + "/velodyne_left.timestamps", ios::in);
        if (infile_left.is_open())
            cout << "Reading velodyne_left timestamps from this txt." << std::endl;
        while (!infile_left.eof())
        {
            getline(infile_left, strline);
            subline = strline.substr(0, strline.find_last_of(' '));
            LidarLeftTimeList.push_back(subline);
        }
        infile_left.close();
        LidarLeftTimeList.pop_back();

        infile_right.open(dataset_path + "/velodyne_right.timestamps", ios::in);
        if (infile_right.is_open())
            cout << "Reading velodyne_right timestamps from this txt." << std::endl;
        while (getline(infile_right, strline))
        {
            subline = strline.substr(0, strline.find_last_of(' '));
            LidarRightTimeList.push_back(subline);
        }
        infile_right.close();
    }

    void lidar::publish(vector<string> TimeList, bool flag)
    {
        //  From SDK extinsics
        vector<float> left_trans = {-0.60072, -0.34077, -0.26837, -0.0053948, -0.041998, -3.1337};
        vector<float> right_trans = {-0.61153, 0.55676, -0.27023, 0.0027052, -0.041999, -3.1357};

        pcl::PointCloud<pcl::PointXYZI> pcls;
        pcls.clear();
        pcls.reserve(1e6);

        string lidar_path;
        size_t lidar_size = TimeList.size() - 1;
        ros::Publisher pubLidar;
        if (!flag)
        {
            pubLidar = n.advertise<sensor_msgs::PointCloud2>("oxford/lidar/left", 10);
            cout << "Publishing left lidar ..." << endl;
        }
        else
        {
            pubLidar = n.advertise<sensor_msgs::PointCloud2>("oxford/lidar/right", 10);
            cout << "Publishing right lidar ..." << endl;
        }

        ros::Rate loop_rate(20);
        for (size_t index = 0; index <= lidar_size; ++index)
        {
            string name = TimeList[index] + ".bin";
            if (!flag)
                lidar_path = left_path + name;
            else
                lidar_path = right_path + name;

            size_t cnt = 0;
            long start, end;
            ifstream filesize(lidar_path, ios::in | ios::binary);
            start = filesize.tellg();
            filesize.seekg(0, ios::end);
            end = filesize.tellg();
            filesize.close();
            cnt = (end - start) / (4 * sizeof(float));

            ifstream input(lidar_path, ios::in | ios::binary);
            if (!input)
            {
                std::cout << "Could not open this pointcloud bin file.\n";
                ros::shutdown();
            }

            for (size_t i = 0; i < cnt; ++i)
            { // N*4, not 4*N!!
                if (input.good() && !input.eof())
                {
                    pcl::PointXYZI pcl;
                    input.read((char *)&pcl.x, sizeof(float));
                    pcls.push_back(pcl);
                }
            }
            for (size_t i = 0; i < cnt; ++i)
                if (input.good() && !input.eof())
                    input.read((char *)&pcls[i].y, sizeof(float));
            for (size_t i = 0; i < cnt; ++i)
                if (input.good() && !input.eof())
                    input.read((char *)&pcls[i].z, sizeof(float));
            for (size_t i = 0; i < cnt; ++i)
                if (input.good() && !input.eof())
                    input.read((char *)&pcls[i].intensity, sizeof(float));

            input.close();

            sensor_msgs::PointCloud2 Lidar_Msg;
            pcl::toROSMsg(pcls, Lidar_Msg);
            Lidar_Msg.header.stamp = sensor::Timestamp2Rostime(TimeList[index]);
            if (!save_flag)
            {
                if (!flag)
                {
                    Lidar_Msg.header.frame_id = lidar_left_frame;
                    TransformBroadcaster(left_trans, Lidar_Msg.header.stamp, stereo_frame, Lidar_Msg.header.frame_id);
                }
                else
                {
                    Lidar_Msg.header.frame_id = lidar_right_frame;
                    TransformBroadcaster(right_trans, Lidar_Msg.header.stamp, stereo_frame, Lidar_Msg.header.frame_id);
                }
                pubLidar.publish(Lidar_Msg);

                pcls.clear();

                ros::spinOnce();
                loop_rate.sleep();
                if (index < lidar_size)
                    loop_rate = sensor::Frequency(TimeList[index + 1], TimeList[index]);
            }
            else
            {
                if (!flag)
                {
                    Lidar_Msg.header.frame_id = lidar_left_frame;
                    oxford_bag.write("oxford/lidar/left", Lidar_Msg.header.stamp, Lidar_Msg);
                }
                else
                {
                    Lidar_Msg.header.frame_id = lidar_right_frame;
                    oxford_bag.write("oxford/lidar/right", Lidar_Msg.header.stamp, Lidar_Msg);
                }
                pcls.clear();
            }
        }
    }

    //------------------------------ class gps--------------------------------------
    gps::gps(const string config_file, const string dataset_path, ros::NodeHandle node)
        : n(node)
    {
        ins_path = dataset_path + "/gps/ins.csv";
        gps_path = dataset_path + "/gps/gps.csv";
        ifstream infile;
        infile.open(gps_path, ios::in);
        if (infile.is_open())
            cout << "Reading gps data from this txt." << std::endl;

        string strline;
        size_t index;
        getline(infile, strline);

        while (getline(infile, strline))
        {
            GpsStructure gpsline;

            index = strline.find_first_of(',');
            gpsline.timestamp = (strline.substr(0, index)).substr(3);
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.num_satellites = stoi(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.latitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.longitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.altitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.latitude_sigma = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.longitude_sigma = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.altitude_sigma = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.northing = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.easting = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            gpsline.down = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            gpsline.utm_zone = strline;

            oxford_gps.push_back(gpsline);
        }
        infile.close();

        infile.open(ins_path, ios::in);
        if (infile.is_open())
            cout << "Reading ins data from this txt." << std::endl;

        getline(infile, strline);

        while (getline(infile, strline))
        {
            InsStructure insline;

            index = strline.find_first_of(',');
            insline.timestamp = (strline.substr(0, index)).substr(3);
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.ins_status = strline.substr(0, index);
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.latitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.longitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.altitude = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.northing = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.easting = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.down = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.utm_zone = strline.substr(0, index);
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.velocity_north = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.velocity_east = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.velocity_down = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.roll = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            index = strline.find_first_of(',');
            insline.pitch = stod(strline.substr(0, index));
            strline = strline.substr(++index);

            insline.yaw = stod(strline);

            oxford_ins.push_back(insline);
        }
        infile.close();
    }

    void gps::publishgps()
    {
        gps_mutex.lock();
        //  From SDK extinsics
        vector<float> gps_trans = {-1.7132, 0.1181, 1.1948, -0.0125, 0.0400, 0.0050};
        ros::Publisher pubGPS = n.advertise<sensor_msgs::NavSatFix>("oxford/gps/gps", 10);

        ros::Rate loop_rate(5);
        cout << "Publishing GPS ..." << endl;
        size_t gps_size = oxford_gps.size() - 1;
        for (size_t i = 0; i <= gps_size; ++i)
        {
            sensor_msgs::NavSatFix gps_position;
            gps_position.header.frame_id = gps_frame;
            gps_position.header.stamp = sensor::Timestamp2Rostime(oxford_gps[i].timestamp);
            gps_position.status.status = 0;
            gps_position.status.service = 1;
            gps_position.latitude = oxford_gps[i].latitude;
            gps_position.longitude = oxford_gps[i].longitude;
            gps_position.altitude = oxford_gps[i].altitude;

            if (!save_flag)
            {
                pubGPS.publish(gps_position);
                TransformBroadcaster(gps_trans, gps_position.header.stamp, stereo_frame, gps_position.header.frame_id);

                ros::spinOnce();
                loop_rate.sleep();
                if (i < gps_size)
                    loop_rate = sensor::Frequency(oxford_gps[i + 1].timestamp, oxford_gps[i].timestamp);
            }
            else
            {
                oxford_bag.write("oxford/gps/gps", gps_position.header.stamp, gps_position);
            }
        }
        gps_mutex.unlock();
    }

    void gps::publishins()
    {
        ins_mutex.lock();
        //  From SDK extinsics
        vector<float> ins_trans = {-1.7132, 0.1181, 1.1948, -0.0125, 0.0400, 0.0050};
        ros::Publisher pubGPS = n.advertise<sensor_msgs::NavSatFix>("oxford/ins/gps", 10);
        ros::Publisher pubImu = n.advertise<sensor_msgs::Imu>("oxford/ins/imu", 10);

        ros::Rate loop_rate(50);
        cout << "Publishing INS ..." << endl;
        size_t ins_size = oxford_ins.size() - 1;
        for (size_t i = 0; i <= ins_size; ++i)
        {
            sensor_msgs::NavSatFix gps;
            gps.header.frame_id = gps_frame;
            gps.header.stamp = sensor::Timestamp2Rostime(oxford_ins[i].timestamp);
            gps.status.status = 0;
            gps.status.service = 1;
            gps.latitude = oxford_ins[i].latitude;
            gps.longitude = oxford_ins[i].longitude;
            gps.altitude = oxford_ins[i].altitude;

            sensor_msgs::Imu imu;
            tf::Quaternion q = tf::createQuaternionFromRPY(oxford_ins[i].roll, oxford_ins[i].pitch, oxford_ins[i].yaw);
            imu.header.frame_id = gps_frame;
            imu.header.stamp = gps.header.stamp;
            imu.orientation.x = q.x();
            imu.orientation.y = q.y();
            imu.orientation.z = q.z();
            imu.orientation.w = q.w();

            if (!save_flag)
            {
                pubGPS.publish(gps);
                pubImu.publish(imu);
                ros::spinOnce();
                loop_rate.sleep();
                if (i < ins_size)
                    loop_rate = sensor::Frequency(oxford_ins[i + 1].timestamp, oxford_ins[i].timestamp);
            }
            else
            {
                oxford_bag.write("oxford/ins/gps", gps.header.stamp, gps);
                oxford_bag.write("oxford/ins/imu", imu.header.stamp, imu);
            }
        }
        ins_mutex.unlock();
    }

    //-------------------------- namespace funcs----------------------------------
    ros::Time Timestamp2Rostime(string timestamp)
    { // UNIX(int64 maybe ms)
        int64_t nsec = stol(timestamp) * 1000;
        return ros::Time().fromNSec(nsec);
    }

    uint64 Timestamp(string timestamp)
    {
        stringstream ss;
        ss << timestamp;
        uint64 timestamp_ = 0;
        ss >> timestamp_;
        return timestamp_;
    }

    double Frequency(string next, string now)
    {
        double nsec = stod(next) - stod(now);
        double freq = 1000000 / nsec;
        return freq;
    }

    void TransformBroadcaster(geometry_msgs::TransformStamped transform)
    {
        oxford_mutex.lock();
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        static_broadcaster.sendTransform(transform);
        oxford_mutex.unlock();
    }

    void TransformBroadcaster(vector<float> trans, ros::Time timestamp, string frameId, string childId)
    {
        oxford_mutex.lock();
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transform = Transform(trans, timestamp, frameId, childId);
        static_broadcaster.sendTransform(static_transform);
        oxford_mutex.unlock();
    }

    void SaveTf2Rosbag(ros::Time timestamp)
    {
        oxford_mutex.lock();
        // From SDK extinsics
        vector<float> trans_left = {0, 0, 0, 0, 0, 0};
        vector<float> trans_centre = {0, 0.119997, 0, 0, 0, 0};
        vector<float> trans_right = {0, 0.239983, 0, 0, 0, 0};
        vector<float> trans_mono_left = {-0.0905, 1.6375, 0.2803, 0.2079, -0.2339, 1.2321};
        vector<float> trans_mono_right = {-0.2587, -1.6810, 0.3226, -0.1961, -0.2469, -1.2675};
        vector<float> trans_mono_rear = {-2.0582, 0.0894, 0.3675, -0.0119, -0.2498, 3.1283};
        vector<float> radar_trans = {-0.71813, 0.12, -0.54479, 0, 0.05, 0};
        vector<float> lidar_left_trans = {-0.60072, -0.34077, -0.26837, -0.0053948, -0.041998, -3.1337};
        vector<float> lidar_right_trans = {-0.61153, 0.55676, -0.27023, 0.0027052, -0.041999, -3.1357};
        vector<float> gps_trans = {-1.7132, 0.1181, 1.1948, -0.0125, 0.0400, 0.0050};

        const size_t size = 9;
        tf2_msgs::TFMessage tf2_msgs;
        geometry_msgs::TransformStamped oxford_transform[size];
        oxford_transform[0] = Transform(trans_centre, timestamp, stereo_frame, stereo_centre_frame);
        oxford_transform[1] = Transform(trans_right, timestamp, stereo_frame, stereo_right_frame);
        oxford_transform[2] = Transform(radar_trans, timestamp, stereo_frame, radar_frame);
        oxford_transform[3] = Transform(lidar_left_trans, timestamp, stereo_frame, lidar_left_frame);
        oxford_transform[4] = Transform(lidar_right_trans, timestamp, stereo_frame, lidar_right_frame);
        oxford_transform[5] = Transform(gps_trans, timestamp, stereo_frame, gps_frame);
        oxford_transform[6] = Transform(trans_mono_left, timestamp, stereo_frame, mono_left_frame);
        oxford_transform[7] = Transform(trans_mono_right, timestamp, stereo_frame, mono_right_frame);
        oxford_transform[8] = Transform(trans_mono_rear, timestamp, stereo_frame, mono_rear_frame);

        for (size_t index = 0; index < size; ++index)
            tf2_msgs.transforms.push_back(oxford_transform[index]);

        oxford_bag.write("/tf", timestamp, tf2_msgs);
        oxford_mutex.unlock();
    }

    geometry_msgs::TransformStamped Transform(vector<float> trans, ros::Time timestamp, string frameId, string childId)
    {
        geometry_msgs::TransformStamped static_transform;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(trans[3], trans[4], trans[5]);
        static_transform.header.stamp = timestamp;
        static_transform.header.frame_id = frameId;
        static_transform.child_frame_id = childId;
        static_transform.transform.translation.x = trans[0];
        static_transform.transform.translation.y = trans[1];
        static_transform.transform.translation.z = trans[2];
        static_transform.transform.rotation.x = q.x;
        static_transform.transform.rotation.y = q.y;
        static_transform.transform.rotation.z = q.z;
        static_transform.transform.rotation.w = q.w;

        return static_transform;
    }

} // namespace sensor
