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

    // Delay before the next reading
    delay(1000);
}
