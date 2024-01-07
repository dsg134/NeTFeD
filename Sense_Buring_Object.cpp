#include <Fire-Detection-Training-4_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// Define I2C bus
#define SDA_PIN 13 // Define your SDA pin
#define SCL_PIN 2 // Define your SCL pin

Adafruit_AMG88xx amg;

const int INPUT_SIZE = 8;
const int OUTPUT_SIZE = 16; // Interpolate data by a factor of: OUTPUT_SIZE / INPUT_SIZE --> This number is limited with however much memory the MCU has

// Pixel grid properties
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float output[OUTPUT_SIZE][OUTPUT_SIZE];

// Hottest point location properties
float maxTemp;
float findMax_x;
float findMax_y;

float NetVisualFireProbability;
float NetThermalFireProbability;
float NetFireProbability;
float DistanceToFire;

// Storage for recovered flame data from second esp32 CAM
struct FireData {
    float FireProbability;
    int X_Position;
    int Y_Position;
};

FireData parsedFireData;

// Probability of detecting fire via thermal imaging
float thermalFireProbability;
const float FIRE_THRESHOLD = 40;

// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

// Image size
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// Private variables
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function definitions
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;

// Make output dataset smoother to make fire location easier
void populateOutputMatrix() {
    float x_ratio = (float)(INPUT_SIZE - 1) / (OUTPUT_SIZE - 1);
    float y_ratio = (float)(INPUT_SIZE - 1) / (OUTPUT_SIZE - 1);
    float store_max = 0;
    float check_for_negatives = 0;

    for (int y = 0; y < OUTPUT_SIZE; y++) {
        for (int x = 0; x < OUTPUT_SIZE; x++) {
            int input_x = int(x * x_ratio);
            int input_y = int(y * y_ratio);
            output[y][x] = pixels[input_y * INPUT_SIZE + input_x];
            if (output[y][x] > store_max) {
              store_max = output[y][x];
              findMax_x = x;
              findMax_y = y;
            }
            if (output[y][x] < 0) {
              check_for_negatives++;
            }
        }
    }
    maxTemp = store_max;
    if (maxTemp <= FIRE_THRESHOLD) {
      thermalFireProbability = exp(-1 * (maxTemp - FIRE_THRESHOLD) * (maxTemp - FIRE_THRESHOLD)); // Assumme a Gaussian Distribution until FIRE_THRESHOLD is met
    }
    else {
      thermalFireProbability = 1;
    }

    // If there are any negative values in the output matrix, the thermal sensor is not properly connected to the I2C bus
    if (check_for_negatives > 0) {
      thermalFireProbability = 0;
    }
}

// Parse esp32 CAM data into readable values
void Parse_Fire_Data(const String& input_data) {
    if (input_data.length() < 60) { // serial fire data should be >60 chars
        parsedFireData.FireProbability = 0.0f;
        parsedFireData.X_Position = 0;
        parsedFireData.Y_Position = 0;
    }
    else {
        String inputData(input_data);

        int openParen = inputData.indexOf('(');
        int closeParen = inputData.indexOf(')');

        String floatValue = inputData.substring(openParen + 1, closeParen);
        float fireValue = floatValue.toFloat();

        int xStart = inputData.indexOf("x:") + 2;
        int xEnd = inputData.indexOf(',', xStart);
        int yStart = inputData.indexOf("y:") + 2;
        int yEnd = inputData.indexOf(',', yStart);
        int xValue = inputData.substring(xStart, xEnd).toInt();
        int yValue = inputData.substring(yStart, yEnd).toInt();

        parsedFireData.FireProbability = fireValue;
        parsedFireData.X_Position = xValue;
        parsedFireData.Y_Position = yValue;
    }
}

// Estimate distance between stereo cameras and fire
float estimate_dist_from_fire(float *cam_1_params, float *cam_2_params) {
  // the param elements are arrays containing information on the stereo cameras
  
  // cam 1 params
  float cam_1_BB_x = cam_1_params[0]; // bounding box (x coord)
  float cam_1_BB_y = cam_1_params[1]; // bounding box (y coord)

  // cam 2 params
  float cam_2_BB_x = cam_2_params[0]; // bounding box (x coord)
  float cam_2_BB_y = cam_2_params[1]; // bounding box (y coord)

  // Intrinsic camera parameters from calibration (cam 1 = cam 2) + stereo geometry
  // All values are measured in millimeters (mm)
  float focal_x = 975.9491;
  float focal_y = 975.3356;
  float princip_x = 506.0773;
  float princip_y = 378.6833;
  float seperation = 77.47; // distance between cam 1 and cam 2

  // Compute the x, y, and z coords of the fire relative to cam 1 as the origin
  float disparity = cam_1_BB_x - cam_2_BB_x;
  float fire_x = seperation * (cam_1_BB_x - princip_x) / disparity;
  float fire_y = seperation * focal_x * (cam_1_BB_y - princip_y) / (focal_y * disparity);
  float fire_z = seperation * focal_x / disparity;

  float dist_to_fire = sqrt(pow(fire_x, 2) + pow(fire_y, 2) + pow(fire_z, 2));
  return dist_to_fire;
}

void setup()
{
    Serial.begin(115200);
    
    //Initialize camera
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    // Begin I2C communication
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize thermal array at 0x69
    bool status;
    status = amg.begin(0x69);
    if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    }
    delay(100); // let sensor boot up

    // ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

void loop()
{
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

     amg.readPixels(pixels);
  
    // Populate the output matrix
    populateOutputMatrix();

    // Print the output matrix for debugging
    /*for (int y = 0; y < OUTPUT_SIZE; y++) {
        for (int x = 0; x < OUTPUT_SIZE; x++) {
            Serial.print(output[y][x]);
            Serial.print("\t");
        }
        Serial.println();
    }*/

    //Print fire data for drone control
    Serial.print("Visual Fire Probability: ");
    Serial.println(NetVisualFireProbability);

    Serial.print("Thermal Fire Probability: ");
    Serial.println(NetThermalFireProbability);

    Serial.print("Measured Temperature: ");
    Serial.println(maxTemp);
    
    Serial.print("Net Fire Probability: ");
    Serial.println(NetFireProbability);

    Serial.print("Distance to Fire: ");
    Serial.println(DistanceToFire);

    Serial.println("BREAK"); // Splits fire data samples
    
    delay(50);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        // ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

        // Use for serial connection debugging between ESP32s
        /*
        Serial.print("First Fire Probability: ");
        Serial.println(bb.value);
        Serial.print("First X Location: ");
        Serial.println(bb.x);
        Serial.print("First Y Location: ");
        Serial.println(bb.y);
        */

        if (Serial.available()) {
          String data = Serial.readStringUntil('\n');
          Parse_Fire_Data(data);

          // Use for serial connection debugging between ESP32s
          /*
          Serial.print("Second Fire Probability: ");
          Serial.println(parsedFireData.FireProbability);
          Serial.print("Second X Location: ");
          Serial.println(parsedFireData.X_Position);
          Serial.print("Second Y Location: ");
          Serial.println(parsedFireData.Y_Position);
          */
        }

        // Combine data from both ESP32 CAM boards
        float avg_FireProbability = (bb.value + parsedFireData.FireProbability) / 2;

        float cam_1_params[2] = {bb.x, bb.y};
        float cam_2_params[2] = {parsedFireData.X_Position, parsedFireData.Y_Position};
        float distance_to_fire = estimate_dist_from_fire(cam_1_params, cam_2_params);
        DistanceToFire = distance_to_fire;

        NetVisualFireProbability = avg_FireProbability;
        NetThermalFireProbability = thermalFireProbability;

        float net_probability_of_fire = avg_FireProbability * thermalFireProbability;
        NetFireProbability = net_probability_of_fire;

        /*
        // Fire Data
        Serial.print("Fire Probability: ");
        Serial.println(avg_FireProbability);
        Serial.print("Thermal Probability: ");
        Serial.println(thermalFireProbability);
        Serial.print("Maximum Temperature: ");
        Serial.println(maxTemp);

        // Fire Tracking
        Serial.print("Net Probability of Fire: ");
        Serial.println(net_probability_of_fire);
        Serial.print("Distance to Fire: ");
        Serial.println(distance_to_fire);
        
        Serial.println("BREAK");
        */
    }
    if (!bb_found) {
        //ei_printf("    No objects found\n");
        //Serial.println("    No objects found");
        NetVisualFireProbability = 0;
        NetThermalFireProbability = 0;
        NetFireProbability = 0;
        DistanceToFire = 0;
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                    result.classification[ix].value);
        Serial.println(result.classification[ix].label);
        Serial.println(result.classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    free(snapshot_buf);
}

bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
