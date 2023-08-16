#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

const int INPUT_SIZE = 8;
const int OUTPUT_SIZE = 12; // Interpolate data by a factor of: OUTPUT_SIZE / INPUT_SIZE --> This number is limit with however much memory the MCU has

// Pixel grid properties
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float output[OUTPUT_SIZE][OUTPUT_SIZE];

// Hottest point location properties
float maxTemp;
float findMax_x;
float findMax_y;

// Probability of detecting fire via thermal imaging
float thermalFireProbability;
const float FIRE_THRESHOLD = 40;

// Fire detection module properties
#define RGB_res_X 1600
#define RGB_res_Y 1200
#define IR_res_X 12
#define IR_res_Y 12
#define seperation_distance 3 // inches

// Storage for recovered flame data from esp32 CAM
struct FireData {
    float FireProbability;
    int X_Position;
    int Y_Position;
};

FireData parsedFireData;

// Parse esp32 CAM data into readable values
void Parse_Fire_Data(const String& input_data) {
    if (input_data.length() == 21) {
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
float estimate_dist_from_fire(float *RGB_params, float *IR_params, float distance) {
  // the param elements are arrays containing information on the RGB and IR cameras

  // RGB_params
  float RGB_FOV_x = RGB_params[0]; // horizontal FOV
  float RGB_FOV_y = RGB_params[1]; // vertical FOV
  float RGB_W = RGB_params[2]; // width of camera image (pixels)
  float RGB_H = RGB_params[3]; // height of camera image (pixels)
  float RGB_fire_center_x = RGB_params[4]; // center of fire x-value (pixels)
  float RGB_fire_center_y = RGB_params[5]; // center of fire y-value (pixels)

  // IR_params
  float IR_FOV_x = IR_params[0]; // horizontal FOV
  float IR_FOV_y = IR_params[1]; // vertical FOV
  float IR_W = IR_params[2]; // width of camera image (pixels)
  float IR_H = IR_params[3]; // height of camera image (pixels)
  float IR_fire_center_x = IR_params[4]; // center of fire x-value (pixels)
  float IR_fire_center_y = IR_params[5]; // center of fire y-value (pixels)

  // Find RGB angles
  float RGB_horiz = ((RGB_fire_center_x - RGB_W / 2) / (RGB_W / 2)) * (RGB_FOV_x / 2);
  float RGB_vert = ((RGB_fire_center_y - RGB_H / 2) / (RGB_H / 2)) * (RGB_FOV_y / 2);

  // Find IR angles
  float IR_horiz = ((IR_fire_center_x - IR_W / 2) / (IR_W / 2)) * (IR_FOV_x / 2);
  float IR_vert = ((IR_fire_center_y - IR_H / 2) / (IR_H / 2)) * (IR_FOV_y / 2);

  // Convert to radians
  RGB_horiz = M_PI * RGB_horiz / 180;
  RGB_vert = M_PI * RGB_vert / 180;
  IR_horiz = M_PI * IR_horiz / 180;
  IR_vert = M_PI * IR_vert / 180;

  // Horizontal distances
  float x_RGB = distance * tan(RGB_horiz);
  float x_IR = distance * tan(IR_horiz);

  // Vertical distances
  float y_RGB = distance * tan(RGB_vert);
  float y_IR = distance * tan(IR_vert);

  // Estimate distance to fire
  float dist_to_fire = sqrt(pow((x_RGB - x_IR), 2) + pow((y_RGB - y_IR), 2) + pow(distance, 2));
  return dist_to_fire;
}

void setup() {
    Serial.begin(115200);
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
    
    delay(600); // Adjust delay so that 1 AMG8833 cycle = 1 ESP32 CAM TX Cycle
    
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        Parse_Fire_Data(data);
        
        Serial.print("Fire Probability: ");
        Serial.println(parsedFireData.FireProbability);
        Serial.print("X Location: ");
        Serial.println(parsedFireData.X_Position);
        Serial.print("Y Location: ");
        Serial.println(parsedFireData.Y_Position);

        Serial.println();
        Serial.print("Combined Probability: ");
        Serial.println(0.3 * thermalFireProbability + 0.7 * parsedFireData.FireProbability);

        float RGB_params[6] = {68, 50, RGB_res_X, RGB_res_Y, parsedFireData.X_Position, parsedFireData.Y_Position};
        float IR_params[6] = {60, 60, IR_res_X, IR_res_Y, findMax_x, findMax_y};
        float dist_to_fire = estimate_dist_from_fire(RGB_params, IR_params, seperation_distance); // The accuracy of this still needs to be tested!!!
        Serial.print("Distance from Fire: ");
        Serial.println(dist_to_fire);
        
        // if ((0.3 * thermalFireProbability + 0.7 * parsedFireData.FireProbability) > 0.8) { // Fire Confidence Threshold = 0.8
            // Send control signals to alert external device of fire
        // }
        
    }
}
