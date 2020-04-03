#include "ximea_ros_cam/ximea_ros_cam.hpp"

namespace ximea_ros_cam {

std::map<std::string, int> XimeaROSCam::ImgFormatMap = {
    {"XI_MONO8",      XI_MONO8},
    {"XI_MONO16",     XI_MONO16},
    {"XI_RGB24",      XI_RGB24},
    {"XI_RGB32",      XI_RGB32},
    {"XI_RGB_PLANAR", XI_RGB_PLANAR},
    {"XI_RAW8",       XI_RAW8},
    {"XI_RAW16",      XI_RAW16}
};

std::map<std::string, int> XimeaROSCam::BytesPerPixelMap = {
    {"XI_MONO8",      1},
    {"XI_MONO16",     2},
    {"XI_RGB24",      3},
    {"XI_RGB32",      4},
    {"XI_RGB_PLANAR", 3},
    {"XI_RAW8",       1},
    {"XI_RAW16",      2}
};

std::map<std::string, std::string> XimeaROSCam::ImgEncodingMap = {
    {"XI_MONO8",      "mono8"},
    {"XI_MONO16",     "mono16"},
    {"XI_RGB24",      "bgr8"},
    {"XI_RGB32",      "bgra8"},
    {"XI_RGB_PLANAR", "not_applicable"},
    {"XI_RAW8",       "mono8"},
    {"XI_RAW16",      "mono16"}
};

// -- Camera Models    -- //
// -- 0 = MQ013CG-E2   -- //
// --                  -- //

std::map<int, int> XimeaROSCam::CamMaxPixelWidth = { {0, 1280} };
std::map<int, int> XimeaROSCam::CamMaxPixelHeight = { {0, 1024} };

XimeaROSCam::XimeaROSCam() : diag_updater{} {
    this->img_count_ = 0;                   // assume 0 images published
    this->cam_framerate_control_ = false;
    this->cam_white_balance_mode_ = 0;
    this->cam_trigger_mode_ = 0;
    this->is_active_ = false;
    this->xi_h_ = NULL;
    this->cam_info_loaded_ = false;
    this->age_min = 0.0;
}

XimeaROSCam::~XimeaROSCam() {
    // Init variables
    XI_RETURN xi_stat;

    ROS_INFO("Shutting down ximea_ros_cam node...");
    // Stop acquisition and close device if handle is available
    if (this->xi_h_ != NULL) {
        // Stop image acquisition
        this->is_active_ = false;
        xi_stat = xiStopAcquisition(this->xi_h_);

        // Close camera device
        xiCloseDevice(this->xi_h_);
        this->xi_h_ = NULL;

        ROS_INFO_STREAM("Closed device: " << this->cam_serialno_);
    }
    ROS_INFO("ximea_ros_cam node shutdown complete.");

    // To avoid warnings
    (void)xi_stat;
}


// onInit() - on the initialization of the nodelet (not the class)
void XimeaROSCam::onInit() {
    // Report start of function
    ROS_INFO("Initializing Nodelet ... ");

    // Execute initialization functions
    this->initNodeHandles();

    // Camera initialization
    this->initCam();

    // Diagnostics
    this->initDiagnostics();

    // Publishers and Subscriptions
    this->initPubs();

    // Timer callback for camera capture (after camera is initialized)
    this->initTimers();

    // Initialize image storage parameters
    this->initStorage();

    // Report end of function
    ROS_INFO("... Nodelet Initialized. Waiting for Input...");
}

// initNodeHandles() - initialize the private/public node handles
void XimeaROSCam::initNodeHandles() {
    // Report start of function
    ROS_INFO("Loading Node Handles ... ");

    // get public/private node handle
    this->public_nh_ = this->getNodeHandle();
    this->private_nh_ = this->getPrivateNodeHandle();

    // Report end of function
    ROS_INFO("... Node Handles Loaded. ");
}

void XimeaROSCam::initDiagnostics() {
    if (this->enable_diagnostics) {
        this->frequency_min =
            this->pub_frequency - this->pub_frequency_tolerance;
        this->frequency_max =
            this->pub_frequency + this->pub_frequency_tolerance;
        this->diag_updater.setHardwareID(this->cam_name_);
        this->cam_pub_diag =
            std::make_shared<diagnostic_updater::TopicDiagnostic>(
                ros::this_node::getNamespace() + "/image_raw",
                this->diag_updater,
                diagnostic_updater::FrequencyStatusParam(
                    &this->frequency_min, &this->frequency_max,
                    0.0, 20),
                diagnostic_updater::TimeStampStatusParam(
                    this->age_min, this->age_max));
    }
}

// initPubs() - initialize the publishers
void XimeaROSCam::initPubs() {
    // Report start of function
    ROS_INFO("Loading Publishers ... ");

    this->cam_img_counter_pub_ = this->private_nh_.advertise<std_msgs::UInt32>(
            "image_count", 0);

    this->private_nh_.param<bool>("publish_xi_image_info",
                                 this->publish_xi_image_info_,
                                 false);
    ROS_INFO_STREAM("publish_xi_image_info: " << this->publish_xi_image_info_);

    if(this->publish_xi_image_info_) {
      this->cam_xi_image_info_pub_ =
        this->private_nh_.advertise<ximea_ros_cam::XiImageInfo>(
          "xi_image_info", 0);
    }

    // Report end of function
    ROS_INFO("... Publishers Loaded. ");
}

// initTimers() - initialize the timers
void XimeaROSCam::initTimers() {
    // Report start of function
    ROS_INFO("Loading Timers ... ");

    // Load camera polling callback timer ((Ensure that with multiple cameras,
    // each time is about 2 seconds spaced apart)
    this->xi_open_device_cb_ =
        this->private_nh_.createTimer(ros::Duration(this->poll_time_),
        boost::bind(&XimeaROSCam::openDeviceCb, this));
    ROS_INFO_STREAM("xi_open_device_cb_: " << this->xi_open_device_cb_);

    // Load camera frame capture callback timer
    this->t_frame_cb_ =
        this->public_nh_.createTimer(ros::Duration(this->poll_time_frame_),
        boost::bind(&XimeaROSCam::frameCaptureCb, this));
    ROS_INFO_STREAM("t_frame_cb_: " << this->t_frame_cb_);

    // Report end of function
    ROS_INFO("... Timers Loaded.");
}

void XimeaROSCam::initStorage() {
    ROS_INFO("Loading Image Storage ... ");

    this->private_nh_.param<std::string>("image_directory",
                                        this->image_directory_,
                                        "NO_PATH");
    ROS_INFO_STREAM("image_directory: " << this->image_directory_);
    this->private_nh_.param<bool>("save_disk",
                                 this->save_disk_,
                                 false);
    ROS_INFO_STREAM("save_disk: " << this->save_disk_);
    this->private_nh_.param<bool>("calib_mode",
                                 this->calib_mode_,
                                 false);
    ROS_INFO_STREAM("calib_mode_: " << this->calib_mode_);


    // Initialize directory paths
    boost::filesystem::path main_dir(this->image_directory_);

    if (this->calib_mode_) {
        this->trigger_sub_ = this->public_nh_.subscribe( "camera/save_image",
            1, &XimeaROSCam::triggerCb, this);

        // directory that holds calibration images
        boost::filesystem::path calib_dir = main_dir /
                                            (this->cam_name_ + "/calib/");
        this->png_path_ = calib_dir.string();
        if (!boost::filesystem::create_directories(calib_dir))
        {
            // failed to create directory, exit ROS and explain.
            ROS_INFO_STREAM("ERROR: unable to create directory: " << this->png_path_);
            ROS_INFO_STREAM("Please make sure that the image_directory "
                         << "parameter is set to a folder with the proper "
                         << "permissions in the config file.");
            ros::shutdown();
        }

        this->save_trigger_ = false;
    }

    // directory that holds video stream images
    if (this->save_disk_) {
        boost::filesystem::path img_stream_dir = main_dir /
                                                 (this->cam_name_ + "/stream/");
        this->bin_path_ = img_stream_dir.string();
        if (!boost::filesystem::create_directories(img_stream_dir))
        {
            // failed to create directory, exit ROS and explain.
            ROS_INFO_STREAM("ERROR: unable to create directory: " << this->bin_path_);
            ROS_INFO_STREAM("Please make sure that the image_directory "
                         << "parameter is set to a folder with the proper "
                         << "permissions in the config file.");
            ros::shutdown();
        }
    }

    ROS_INFO("Image Storage Loaded.");
}

void XimeaROSCam::initCam() {
    ROS_INFO("Loading Camera Configuration");

    // Assume that all of the config is embedded in the camera private namespace
    // Load all parameters and store them into their corresponding vars

    //      -- apply camera name --
    this->private_nh_.param( "cam_name", this->cam_name_,
        std::string("INVALID"));
    ROS_INFO_STREAM("cam_name: " << this->cam_name_);
    //      -- apply camera specific parameters --
    this->private_nh_.param( "serial_no", this->cam_serialno_,
        std::string("INVALID"));
    ROS_INFO_STREAM("serial number: " << this->cam_serialno_);
    this->private_nh_.param( "frame_id", this->cam_frameid_,
        std::string("INVALID"));
    ROS_INFO_STREAM("frame id: " << this->cam_frameid_);
    this->private_nh_.param( "calib_file", this->cam_calib_file_,
        std::string("INVALID"));
    ROS_INFO_STREAM("calibration file: " << this->cam_calib_file_);
    this->private_nh_.param("poll_time", this->poll_time_, -1.0f);
    ROS_INFO_STREAM("poll_time: " << this->poll_time_);
    this->private_nh_.param("poll_time_frame", this->poll_time_frame_, 0.0f);
    ROS_INFO_STREAM("poll_time_frame: " << this->poll_time_frame_);

    // Diagnostics
    this->private_nh_.param("enable_diagnostics", this->enable_diagnostics,
        true);
    ROS_INFO_STREAM("enable_diagnostics: " << this->enable_diagnostics);
    this->private_nh_.param("pub_frequency", this->pub_frequency, 10.0);
    ROS_INFO_STREAM("pub_frequency: " << this->pub_frequency);
    this->private_nh_.param("pub_frequency_tolerance",
        this->pub_frequency_tolerance, 0.3);
    ROS_INFO_STREAM("pub_frequency_tolerance: " << this->pub_frequency_tolerance);
    this->private_nh_.param("data_age_max", this->age_max, 0.1);
    ROS_INFO_STREAM("data_age_max: " << this->age_max);

    //      -- apply compressed image parameters (from image_transport) --
    this->private_nh_.param( "image_transport_compressed_format",
        this->cam_compressed_format_, std::string("INVALID"));
    ROS_INFO_STREAM("image_transport_compressed_format: "
        << this->cam_compressed_format_);
    this->private_nh_.param( "image_transport_compressed_jpeg_quality",
        this->cam_compressed_jpeg_quality_, -1);
    ROS_INFO_STREAM("image_transport_compressed_jpeg_quality: "
        << this->cam_compressed_jpeg_quality_);
    this->private_nh_.param( "image_transport_compressed_png_level",
        this->cam_compressed_png_level_, -1);
    ROS_INFO_STREAM("image_transport_compressed_png_level: "
        << this->cam_compressed_png_level_);

    //      -- apply image format parameters --
    this->private_nh_.param( "format", this->cam_format_,
        std::string("INVALID"));
    ROS_INFO_STREAM("format: " << this->cam_format_);
    this->cam_format_int_ = ImgFormatMap[this->cam_format_];
    ROS_INFO_STREAM("format_int: " << this->cam_format_int_);
    this->cam_bytesperpixel_ = BytesPerPixelMap[this->cam_format_];
    ROS_INFO_STREAM("cam_bytesperpixel_: " << this->cam_bytesperpixel_);
    this->cam_encoding_ = ImgEncodingMap[this->cam_format_];
    ROS_INFO_STREAM("cam_encoding_: " << this->cam_encoding_);

    //      -- apply bandwidth parameters --
    this->private_nh_.param("num_cams_in_bus", this->cam_num_in_bus_, -1);
    ROS_INFO_STREAM("cam_num_in_bus_: " << this->cam_num_in_bus_);
    this->private_nh_.param("bw_safetyratio", this->cam_bw_safetyratio_, -1.0f);
    ROS_INFO_STREAM("cam_bw_safetyratio_: " << this->cam_bw_safetyratio_);

    //      -- apply triggering parameters --
    this->private_nh_.param("cam_trigger_mode", this->cam_trigger_mode_, -1);
    ROS_INFO_STREAM("cam_trigger_mode_: " << this->cam_trigger_mode_);
    this->private_nh_.param("hw_trigger_edge", this->cam_hw_trigger_edge_, -1);
    ROS_INFO_STREAM("cam_hw_trigger_edge_: " << this->cam_hw_trigger_edge_);

    //      -- apply framerate (software cap) parameters --
    this->private_nh_.param( "frame_rate_control",
        this->cam_framerate_control_, false);
    ROS_INFO_STREAM("cam_framerate_control_: " << this->cam_framerate_control_);
    this->private_nh_.param("frame_rate_set", this->cam_framerate_set_, -1);
    ROS_INFO_STREAM("cam_framerate_set_: " << this->cam_framerate_set_);
    this->private_nh_.param( "img_capture_timeout",
        this->cam_img_cap_timeout_, -1);
    ROS_INFO_STREAM("cam_img_cap_timeout_: " << this->cam_img_cap_timeout_);

    //      -- apply exposure parameters --
    this->private_nh_.param("auto_exposure", this->cam_autoexposure_, false);
    ROS_INFO_STREAM("cam_autoexposure_: " << this->cam_autoexposure_);
    this->private_nh_.param("manual_gain", this->cam_manualgain_, -1.0f);
    ROS_INFO_STREAM("cam_manualgain_: " << this->cam_manualgain_);
    this->private_nh_.param("exposure_time", this->cam_exposure_time_, -1);
    ROS_INFO_STREAM("cam_exposure_time_: " << this->cam_exposure_time_);
    this->private_nh_.param( "auto_exposure_priority",
        this->cam_autoexposure_priority_, -1.0f);
    ROS_INFO_STREAM("cam_autoexposure_priority_: "
        << this->cam_autoexposure_priority_);
    this->private_nh_.param("auto_time_limit", this->cam_autotime_limit_, -1);
    ROS_INFO_STREAM("cam_autotime_limit_: " << this->cam_autotime_limit_);
    this->private_nh_.param( "auto_gain_limit",
        this->cam_autogain_limit_, -1.0f);
    ROS_INFO_STREAM("cam_autogain_limit_: " << this->cam_autogain_limit_);

    //      -- apply white balance parameters --
    this->private_nh_.param( "white_balance_mode",
        this->cam_white_balance_mode_, -1);
    ROS_INFO_STREAM("cam_white_balance_mode_: "
        << this->cam_white_balance_mode_);
    this->private_nh_.param( "white_balance_coef_red",
        this->cam_white_balance_coef_r_, -1.0f);
    ROS_INFO_STREAM("cam_white_balance_coef_r_: "
        << this->cam_white_balance_coef_r_);
    this->private_nh_.param( "white_balance_coef_green",
        this->cam_white_balance_coef_g_, -1.0f);
    ROS_INFO_STREAM("cam_white_balance_coef_g_: "
        << this->cam_white_balance_coef_g_);
    this->private_nh_.param( "white_balance_coef_blue",
        this->cam_white_balance_coef_b_, -1.0f);
    ROS_INFO_STREAM("cam_white_balance_coef_b_: "
        << this->cam_white_balance_coef_b_);

    //      -- apply ROI parameters --
    this->private_nh_.param("roi_left", this->cam_roi_left_, -1);
    ROS_INFO_STREAM("cam_roi_left_: " << this->cam_roi_left_);
    this->private_nh_.param("roi_top", this->cam_roi_top_, -1);
    ROS_INFO_STREAM("cam_roi_top_: " << this->cam_roi_top_);
    this->private_nh_.param("roi_width", this->cam_roi_width_, -1);
    ROS_INFO_STREAM("cam_roi_width_: " << this->cam_roi_width_);
    this->private_nh_.param("roi_height", this->cam_roi_height_, -1);
    ROS_INFO_STREAM("cam_roi_height_: " << this->cam_roi_height_);

    // Other basic init values
    this->is_active_ = false;
    this->xi_h_ = NULL;

    // Set compression parameters prior to declaring an image_transport
    // So the dynamic reconfigure initializes with these values
    this->private_nh_.setParam("/image_raw/compressed/format",
                               this->cam_compressed_format_);
    this->private_nh_.setParam("/image_raw/compressed/jpeg_quality",
                               this->cam_compressed_jpeg_quality_);
    this->private_nh_.setParam("/image_raw/compressed/png_level",
                               this->cam_compressed_png_level_);

    // Setup image transport (publishing) and camera info topics
    image_transport::ImageTransport it(this->private_nh_);
    this->cam_pub_ = it.advertise("image_raw", 1);

    // only load and publish calib file if it isn't empty
    // assume camera info is not loaded
    // Setup camera info manager for calibration
    this->cam_info_loaded_ = false;
    this->cam_info_manager_ =
        boost::make_shared<camera_info_manager::CameraInfoManager>
                    (this->private_nh_, this->cam_name_);
    if (this->cam_info_manager_->loadCameraInfo(this->cam_calib_file_)) {
        this->cam_info_loaded_ = true;
    }
    // loaded camera info properly
    if (this->cam_info_loaded_) {
        // advertise
        this->cam_info_pub_ =
            this->private_nh_.advertise<sensor_msgs::CameraInfo>(
                "camera_info", 1);
    }

    // Enable auto bandwidth calculation to ensure bandwidth limiting and
    // framerate setting are supported
    xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);
}

void XimeaROSCam::openCam() {

    // Init variables
    XI_RETURN xi_stat;

    // leave if there isn't a valid handle
    if (this->xi_h_ == NULL) { return; }

    // Apply parameters to camera
    //      -- Set image format --
    xi_stat = xiSetParamInt(this->xi_h_,
                            XI_PRM_IMAGE_DATA_FORMAT,
                            this->cam_format_int_);

    // //      -- Set auto white balance if requested --
    // if (this->cam_auto_white_balance_) {
    //     xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 1);
    // } else {
    //     xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
    // }

    //      -- White balance mode --
    // Note: Setting XI_PRM_MANUAL right before or after setting coeffs
    // actually overrides the coefficients! This is because calculating
    // the manual coeffs takes time, so when the coefficients are set,
    // they will be overwritten once the manual coeff values are calculated.
    // This also is the same when XI_PRM_MANUAL is set to 0 as well.
    if (this->cam_white_balance_mode_ == 2) {
        ROS_INFO_STREAM("WHITE BALANCE MODE SET TO AUTO.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 1);
    } else if (this->cam_white_balance_mode_ == 1) {
        ROS_INFO_STREAM("WHITE BALANCE MODE SET TO APPLY COEFFS.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KR,
                                  this->cam_white_balance_coef_r_);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KG,
                                  this->cam_white_balance_coef_g_);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KB,
                                  this->cam_white_balance_coef_b_);
    } else if (this->cam_white_balance_mode_ == 0) {
        ROS_INFO_STREAM("WHITE BALANCE MODE SET TO NONE.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
    } else {
        // should not be here!
        ROS_INFO_STREAM("WHITE BALANCE MODE IS NOT 0 TO 2!");
    }

    // error handling
    //TODO(carloswanguw) Add error handling for xi_stat
    // errorHandling(stat, "image_format");

    // //      -- Set image trigger mode --
    //TODO(carloswanguw) Add hardware triggering mode here
    // // Setup trigger mode
    // xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    // // If software trigger, this is to send a software trigger to the cam
    // xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOFTWARE, 1);
    // errorHandling(xi_stat, "Error During triggering");

    // Camera hardware trigger mode enabled?
    if (this->cam_trigger_mode_ == 2) {
        if (this->cam_hw_trigger_edge_ == 0) {
            // Select trigger to be rising edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);
        } else if (this->cam_hw_trigger_edge_ == 1) {
            // Select trigger to be falling edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_FALLING);
        } else { // default to rising
            // Select trigger to be rising edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);

        }

        // Select input pin 1 to be for GP input trigger
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_GPI_SELECTOR, 1);
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
    } else if (this->cam_trigger_mode_ == 1) {
        // Select software triggering
        // NOT FULLY IMPLEMENTED YET
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    } else {
        // Disable any triggering
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOURCE, XI_TRG_OFF);
    }

    //      -- Set exposure --
    // If auto exposure is set to 0, then set manual exposure, otherwise set
    // auto exposure.
    //      -- Set manual exposure --
    if (!this->cam_autoexposure_) {
        ROS_INFO_STREAM("Setting manual exposure: EXPOSURE AMOUNT: " <<
                        this->cam_exposure_time_ << " GAIN: " <<
                        this->cam_manualgain_);
        // manual exposure
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_AEAG,
                                0);
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_EXPOSURE,
                                this->cam_exposure_time_);
        // exposure gain limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_GAIN,
                                  this->cam_manualgain_);
    //      -- Set auto exposure --
    } else {
        ROS_INFO_STREAM("Setting auto exposure: EXPOSURE TIME LIMIT: " <<
                        this->cam_autotime_limit_ << " GAIN LIMIT: " <<
                        this->cam_autogain_limit_ << " AUTO PRIORITY: " <<
                        this->cam_autoexposure_priority_);
        // auto exposure
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_AEAG,
                                1);
        // auto priority
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_EXP_PRIORITY,
                                  this->cam_autoexposure_priority_);
        // auto exposure time limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_AE_MAX_LIMIT,
                                  this->cam_autotime_limit_);
        // auto exposure gain limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_AG_MAX_LIMIT,
                                  this->cam_autogain_limit_);
    }


    //      -- Set region of interest --
    int max_cam_width = CamMaxPixelWidth[this->cam_model_];
    ROS_INFO_STREAM("MAX WIDTH: " << max_cam_width);
    int max_cam_height = CamMaxPixelHeight[this->cam_model_];
    ROS_INFO_STREAM("MAX HEIGHT: " << max_cam_height);

    // Check bounds
    if (this->cam_roi_left_ < 0 || this->cam_roi_left_ > max_cam_width ||
        this->cam_roi_top_ < 0 || this->cam_roi_top_ > max_cam_height ||
        this->cam_roi_width_ < 0 || this->cam_roi_width_ > max_cam_width ||
        this->cam_roi_height_ < 0 || this->cam_roi_height_ > max_cam_height ||
        this->cam_roi_left_ + this->cam_roi_width_ > max_cam_width ||
        this->cam_roi_top_ + this->cam_roi_height_ > max_cam_height) {
        // Out of bounds, throw error here
        return;
    }

    // Set ROI
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_WIDTH, this->cam_roi_width_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_HEIGHT, this->cam_roi_height_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_OFFSET_X, this->cam_roi_left_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_OFFSET_Y, this->cam_roi_top_);
    // xiGetParamInt(xiH_, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &tmp);
    // std::cout << "width increment " << tmp << std::endl;

    //      -- Limit bandwidth based on amount of cameras per hub --
    // If

    // Compute available bandwidth for this camera
    int avail_bw = 0;           // Mbits per second
    xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_AVAILABLE_BANDWIDTH, &avail_bw);

    // If we have more than one camera per bus/controller, we divide the
    // available bw to accomodate for the same amount of cameras
    if (this->cam_num_in_bus_ > 1) {
        avail_bw = (int) ((double)avail_bw / (double)this->cam_num_in_bus_);
    }

    // Set bandwidth limit for camera and apply a safety ratio
    ROS_INFO_STREAM("Limiting bandwidth to: " <<
            (int)((float)avail_bw*this->cam_bw_safetyratio_) << " Mbits/sec");
    xi_stat = xiSetParamInt(this->xi_h_,
                            XI_PRM_LIMIT_BANDWIDTH,
                            (int)((float)avail_bw*this->cam_bw_safetyratio_));
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_LIMIT_BANDWIDTH_MODE , XI_ON);


    //      -- Framerate control  --
    // For information purposes, obtain min and max calculated possible fps
    xi_stat = xiGetParamFloat(this->xi_h_,
                              XI_PRM_FRAMERATE XI_PRM_INFO_MIN,
                              &this->min_fps_);
    xi_stat = xiGetParamFloat(this->xi_h_,
                              XI_PRM_FRAMERATE XI_PRM_INFO_MAX,
                              &this->max_fps_);

    // If we are not in trigger mode, determine if we want to limit fps
    if (this->cam_trigger_mode_ == 0) {
        if (this->cam_framerate_control_) {
            ROS_INFO_STREAM("Setting frame rate control to: " <<
                        this->cam_framerate_set_ << " Hz");

            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_ACQ_TIMING_MODE,
                                    XI_ACQ_TIMING_MODE_FRAME_RATE);
            // Apply frame rate (we assume MQ camera here)
            xi_stat = xiSetParamInt(this->xi_h_,
                                      XI_PRM_FRAMERATE,
                                      this->cam_framerate_set_);
        } else {
            // default to free run
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_ACQ_TIMING_MODE,
                                    XI_ACQ_TIMING_MODE_FREE_RUN);
        }
    }

    //      -- Optimize transport buffer commit/size based on payload  --
    // // For usb controllers that can handle it...
    // src: https://www.ximea.com/support/wiki/apis/Linux_USB30_Support
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, 32);
    // xiGetParamInt( handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MAX,
    //  &buffer_size);
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, buffer_size);

    // // For high frame rate performance
    // src: https://www.ximea.com/support/wiki/usb3/...
    //      ...How_to_optimize_software_performance_on_high_frame_rates

    //      -- Start camera acquisition --
    ROS_INFO("Starting Acquisition...");
    xi_stat = xiStartAcquisition(this->xi_h_);
    ROS_INFO("Acquisition started...");

    this->is_active_ = true;                    // set active to be true

    // To avoid warnings
    (void)xi_stat;
}

void XimeaROSCam::openDeviceCb() {
    XI_RETURN xi_stat;

    ROS_INFO_STREAM("Polling Ximea Cam. Serial #: " << this->cam_serialno_);

    xi_stat = xiOpenDeviceBy(XI_OPEN_BY_SN,
            this->cam_serialno_.c_str(),
            &this->xi_h_);

    if (xi_stat == XI_OK && this->xi_h_ != NULL) {
        ROS_INFO_STREAM("Poll successful. Loading serial #: "
                        << this->cam_serialno_);
        this->xi_open_device_cb_.stop();
        XimeaROSCam::openCam();
    }

    // To avoid warnings
    (void)xi_stat;
}

// Start aquiring data
void XimeaROSCam::frameCaptureCb() {
    // Init variables
    XI_RETURN xi_stat;
    XI_IMG xi_img;
    char *img_buffer;
    int img_buf_size;
    ros::Time timestamp;
    std::string time_str;

    xi_img.size = sizeof(XI_IMG);
    xi_img.bp = NULL;
    xi_img.bp_size = 0;

    // Acquisition started
    if (this->is_active_) {
        // Acquire image
        xi_stat = xiGetImage(this->xi_h_,
                             this->cam_img_cap_timeout_,
                             &xi_img);
        // Add timestamp
        // Here we input the CPU time (ros::Time::now())
        // During simulation, we can use WALLTIME
        timestamp = ros::Time::now();

        // Was the image retrieval successful?
        if (xi_stat == XI_OK) {
            ROS_INFO_STREAM_THROTTLE(3,
                "Capturing image from Ximea camera serial no: "
                << this->cam_serialno_
                << ". WxH: "
                << xi_img.width
                << " x "
                << xi_img.height << ".");
            // Setup image
            img_buffer = reinterpret_cast<char *>(xi_img.bp);
            img_buf_size = xi_img.width * xi_img.height
                           * this->cam_bytesperpixel_;

            // Correctly format time as a string
            time_str = this->formatTimeString(timestamp.toBoost());

            // Save image as binary file
            if (this->save_disk_) {
                std::string bin_path = this->bin_path_ + time_str + "_" +
                                       this->cam_name_ + ".bin";

                if (this->saveToDisk(img_buffer, img_buf_size, bin_path)) {
                    ROS_INFO_STREAM("Saved image to: " << bin_path);
                }
                else {
                    ROS_INFO_STREAM("Failed to save image: " << bin_path);
                }
            }
            // Publish as ROS message
            else {
                sensor_msgs::Image img;
                // Populate ROS message
                sensor_msgs::fillImage(img,
                                       this->cam_encoding_,
                                       xi_img.height,
                                       xi_img.width,
                                       xi_img.width * this->cam_bytesperpixel_,
                                       img_buffer);
                img.header.frame_id = this->cam_frameid_;
                img.header.stamp = timestamp;

                // Publish image
                this->cam_pub_.publish(img);
                if (this->enable_diagnostics) {
                    cam_pub_diag->tick(timestamp);
                    diag_updater.update();
                }

                // Publish camera calibration info if camera info is loaded
                if (this->cam_info_loaded_) {
                    sensor_msgs::CameraInfo cam_info =
                        this->cam_info_manager_->getCameraInfo();
                        // reset frame id
                    cam_info.header.frame_id = this->cam_frameid_;
                    cam_info.header.stamp = timestamp;
                    this->cam_info_pub_.publish(cam_info);
                }

                // Publish image counter
                // Note that header.seq does this, but it is depreciated and
                // will be removed in ROS 2. Therefore here we did this instead.
                std_msgs::UInt32 icount;
                this->img_count_++;                 // increment
                icount.data = this->img_count_;
                this->cam_img_counter_pub_.publish(icount);
            }

            // Compress and save images if triggered and in calibration mode
            if (this->save_trigger_ && this->calib_mode_) {
                this->save_trigger_ = false;

                time_str = this->formatTimeString(timestamp.toBoost());

                if (this->image_directory_ != std::string("NO_PATH")) {
                    std::string png_path = this->png_path_ + time_str + "_" +
                                           this->cam_name_ + ".png";

                    if (this->saveOnTrigger(img_buffer,
                                            xi_img.height,
                                            xi_img.width,
                                            png_path)) {
                        ROS_INFO_STREAM("Saved image to: " << png_path);
                    }
                    else {
                        ROS_INFO_STREAM("Failed to save image: " << png_path);
                    }
                }
                else {
                    ROS_INFO_STREAM("Directory path not set!");
                }
            }
        }
        else {

        }

        // If active, publish xiGetImage info to ROS message
        if(this->publish_xi_image_info_) {
          ximea_ros_cam::XiImageInfo xiImageInfoMsg;
          xiImageInfoMsg.header.frame_id = this->cam_frameid_;
          xiImageInfoMsg.header.stamp = timestamp;
          xiImageInfoMsg.size = xi_img.size;
          xiImageInfoMsg.bp_size = xi_img.bp_size;
          xiImageInfoMsg.frm = xi_img.frm;
          xiImageInfoMsg.width = xi_img.width;
          xiImageInfoMsg.height = xi_img.height;
          xiImageInfoMsg.nframe = xi_img.nframe;
          xiImageInfoMsg.tsSec = xi_img.tsSec;
          xiImageInfoMsg.tsUSec = xi_img.tsUSec;
          xiImageInfoMsg.GPI_level = xi_img.GPI_level;
          xiImageInfoMsg.black_level = xi_img.black_level;
          xiImageInfoMsg.padding_x = xi_img.padding_x;
          xiImageInfoMsg.AbsoluteOffsetX = xi_img.AbsoluteOffsetX;
          xiImageInfoMsg.AbsoluteOffsetY = xi_img.AbsoluteOffsetY;
          xiImageInfoMsg.exposure_time_us = xi_img.exposure_time_us;
          xiImageInfoMsg.gain_db = xi_img.gain_db;
          xiImageInfoMsg.acq_nframe = xi_img.acq_nframe;
          xiImageInfoMsg.image_user_data = xi_img.image_user_data;
          // xiGetImageMsg.exposure_sub_times_us = (unsigned int) xi_img.exposure_sub_times_us;
          this->cam_xi_image_info_pub_.publish(xiImageInfoMsg);
        }
    }

    // To avoid warnings
    (void)xi_stat;
}

// Directly write image data to the disk as a binary file
bool XimeaROSCam::saveToDisk(char *img_buffer,
                                     int img_size,
                                     std::string filename) {
    std::fstream output_file(filename,
                             std::fstream::out | std::fstream::binary);

    output_file.write(img_buffer, img_size);
    output_file.close();

    if (!output_file) {
        return false;
    }
    return true;
}

// Save images on trigger with PNG compression
bool XimeaROSCam::saveOnTrigger(char *img_buffer,
                                        int img_h,
                                        int img_w,
                                        std::string filename) {
    cv::Mat cv_mat = cv::Mat(img_h, img_w, CV_8UC3, img_buffer);
    // Use PNG compression and least amount of compression possible
    // PNG compression scales from 0 (least) to 9 (most)
    std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 0};

    if (!cv::imwrite(filename, cv_mat, compression_params)) {
        return false;
    }
    return true;
}

// Format ROS timestamp into desired string format
// Format is YYYYMMDD_HHMMSS_uS
std::string XimeaROSCam::formatTimeString
        (boost::posix_time::ptime timestamp) {
    boost::posix_time::time_facet *facet = new boost::posix_time::time_facet();
    // Format is YYYYMMDD_HHMMSS_fractionalSeconds
    facet->format("%Y%m%d_%H%M%S_%f");

    std::stringstream stream;
    stream.imbue(std::locale(std::locale::classic(), facet));
    stream << timestamp;

    std::string formatted_time = stream.str();
    boost::erase_all(formatted_time, "."); // remove decimal
    return formatted_time;
}

// Set save_trigger_ flag
void XimeaROSCam::triggerCb(const std_msgs::Empty::ConstPtr& msg) {
    this->save_trigger_ = true;

    // To avoid warnings
    (void)msg;
}


} // NAMESPACE ximea_ros_cam

PLUGINLIB_EXPORT_CLASS(ximea_ros_cam::XimeaROSCam, nodelet::Nodelet);
