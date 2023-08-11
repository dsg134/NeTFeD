#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

const int INPUT_SIZE = 8;
const int OUTPUT_SIZE = 13; // Interpolate data by a factor of: OUTPUT_SIZE / INPUT_SIZE --> 2x

// Pixel grid properties
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float output[OUTPUT_SIZE][OUTPUT_SIZE];

// Hottest point location properties
float maxTemp;
float findMax_x;
float findMax_y;

// Probability of detecting fire via thermal imaging
float thermalFireProbability;
const float FIRE_THRESHOLD = 32;

// Fire detection module properties
#define RGB_res_X 1600
#define RGB_res_Y 1200
#define IR_res_X 13
#define IR_res_Y 13
#define distance 3 //inches

void setup() {
    Serial.begin(9600);
    Serial.println(F("AMG88xx pixels"));

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    delay(100); // let sensor boot up
}

// Make output dataset smoother to make fire location easier
void populateOutputMatrix() {
    float x_ratio = (float)(INPUT_SIZE - 1) / (OUTPUT_SIZE - 1);
    float y_ratio = (float)(INPUT_SIZE - 1) / (OUTPUT_SIZE - 1);
    float store_max = 0;

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
        }
    }
    maxTemp = store_max;
    if (maxTemp <= FIRE_THRESHOLD) {
      thermalFireProbability = exp(-1 * (maxTemp - FIRE_THRESHOLD) * (maxTemp - FIRE_THRESHOLD)); // Assumme a Gaussian Distribution until FIRE_THRESHOLD is met
    }
    else {
      thermalFireProbability = 1;
    }
}

// Estimate module's distance from the fire using triangulation
// if this doesn't work, maybe using PID control to change drone position until fire is centered on both sensor matricies before ejection is a good idea
float estimate_dist_from_fire(double *RGB_params, double *IR_params, double distance) {
  // the param elements are arrays containing information on the RGB and IR cameras

  // RGB_params
  double RGB_FOV_x = RGB_params[0]; // horizontal FOV
  double RGB_FOV_y = RGB_params[1]; // vertical FOV
  double RGB_W = RGB_params[2]; // width of camera image (pixels)
  double RGB_H = RGB_params[3]; // height of camera image (pixels)
  double RGB_fire_center_x = RGB_params[4]; // center of fire x-value (pixels)
  double RGB_fire_center_y = RGB_params[5]; // center of fire y-value (pixels)

  // IR_params
  double IR_FOV_x = IR_params[0]; // horizontal FOV
  double IR_FOV_y = IR_params[1]; // vertical FOV
  double IR_W = IR_params[2]; // width of camera image (pixels)
  double IR_H = IR_params[3]; // height of camera image (pixels)
  double IR_fire_center_x = IR_params[4]; // center of fire x-value (pixels)
  double IR_fire_center_y = IR_params[5]; // center of fire y-value (pixels)

  // Find RGB angles
  double RGB_horiz = ((RGB_fire_center_x - RGB_W / 2) / (RGB_W / 2)) * (RGB_FOV_x / 2);
  double RGB_vert = ((RGB_fire_center_y - RGB_H / 2) / (RGB_H / 2)) * (RGB_FOV_y / 2);

  // Find IR angles
  double IR_horiz = ((IR_fire_center_x - IR_W / 2) / (IR_W / 2)) * (IR_FOV_x / 2);
  double IR_vert = ((IR_fire_center_y - IR_H / 2) / (IR_H / 2)) * (IR_FOV_y / 2);

  // Convert to radians
  RGB_horiz = M_PI * RGB_horiz / 180;
  RGB_vert = M_PI * RGB_vert / 180;
  IR_horiz = M_PI * IR_horiz / 180;
  IR_vert = M_PI * IR_vert / 180;

  // Horizontal distances
  double x_RGB = distance * tan(RGB_horiz);
  double x_IR = distance * tan(IR_horiz);

  // Vertical distances
  double y_RGB = distance * tan(RGB_vert);
  double y_IR = distance * tan(IR_vert);

  // Estimate distance to fire
  double dist_to_fire = sqrt(pow((x_RGB - x_IR), 2) + pow((y_RGB - y_IR), 2) + pow(distance, 2));
  return dist_to_fire;
}

void loop() {
    // Read all the pixels from the AMG8833 sensor
    amg.readPixels(pixels);

    // Populate the output matrix
    populateOutputMatrix();

    // Print the output matrix for debugging
    for (int y = 0; y < OUTPUT_SIZE; y++) {
        for (int x = 0; x < OUTPUT_SIZE; x++) {
            Serial.print(output[y][x]);
            Serial.print("\t");
        }
        Serial.println();
    }

    // Display information about possible fire location
    Serial.print("Highest Temperature: ");
    Serial.println(maxTemp);
    Serial.print("Pixel Location: ");
    Serial.print("x = ");
    Serial.print(findMax_x);
    Serial.print(" ");
    Serial.print("y = ");
    Serial.print(findMax_y);
    Serial.println();
    Serial.print("Probability of Fire: ");
    Serial.print(thermalFireProbability);
    Serial.println();
    Serial.println();

    if (Serial.available()) {
        float receivedValue = Serial.parseFloat();
        Serial.print("Received Classification Value: ");
        Serial.println(receivedValue);
        Serial.print("Combined Probability: ");
        Serial.println(0.7 * receivedValue + 0.3 * thermalFireProbability);

        //also send the x and y locations for the fire center
        //this is needed for triangulation --> for findRGB_x and findRGB_y
        double RGB_params[6] = {68, 50, RGB_res_X, RGB_res_Y, findRGB_x, findRGB_y};
        double IR_params[6] = {60, 60, IR_res_X, IR_res_Y, findMax_x, findMax_y};
        double dist_to_fire = estimate_dist_from_fire(RGB_params, IR_params, distance);
        Serial.print("Distance from Fire: ");
        Serial.println(dist_to_fire);
    }

    // Delay before the next reading
    delay(1000);
}
