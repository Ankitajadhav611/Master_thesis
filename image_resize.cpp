#include <JPEGDecoder.h>
#include <lab_human_detection_inferencing.h>
#include <ArduCAM.h>
#include <Wire.h>
#include <SPI.h>
#include <iostream>


#define FRAME_BUFFER_COLS           320 
#define FRAME_BUFFER_ROWS           240
//#define FRAME_BYTE_SIZE             3

#define CUTOUT_COLS                 EI_CLASSIFIER_INPUT_WIDTH
#define CUTOUT_ROWS                 EI_CLASSIFIER_INPUT_HEIGHT
const int cutout_row_start = (FRAME_BUFFER_ROWS - CUTOUT_ROWS) / 2;
const int cutout_col_start = (FRAME_BUFFER_COLS - CUTOUT_COLS) / 2;

uint16_t* pixel_buffer;
char* jpeg_buffer;
//using namespace ei::image::processing;
const int CS = 34;
const int CAM_POWER_ON = A10;
ArduCAM myCAM(OV5642, CS);

//sleep after each inference 
const int sleepTimeS = 5;

static int k = 0;
// static int j = 100;

//ei print 
// Edge Impulse standardized print method, used for printing results after inference
void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

//variable to store 
#define FORMAT_SPIFFS_IF_FAILED true
char pname[20];
byte buf[256];
uint8_t temp = 0, temp_last = 0;
uint32_t length = 0;
bool is_header = false;
static int i = 0;

void capture_resize_image(uint16_t*& pixel_buffer, size_t width, size_t height){
  
  uint32_t jpeg_length = 0;

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();

  // Start capture
  myCAM.start_capture();
  Serial.println(F("Star Capture"));

  while (!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
  Serial.println(F("Capture Done."));
  delay(50);

  // Clear the capture done flag
  myCAM.clear_fifo_flag();

  jpeg_length = myCAM.read_fifo_length();
  jpeg_buffer = static_cast<char*>(heap_caps_malloc(jpeg_length * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
  Serial.println(F("jpeg length"));
  Serial.println(jpeg_length);
  // if (jpeg_length > 20000) {
  //   Serial.println(F("Error: buffer not large enough to hold image"));
  //   return;
  // }

  if (jpeg_length == 0){
    Serial.println("No data in Arducam FIFO buffer");
    return;
  }

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  uint8_t temp;
  for (int index = 0; index < jpeg_length; index++){
    temp = SPI.transfer(0x00);
    jpeg_buffer[index] = temp;
  }

  delayMicroseconds(15);
  myCAM.CS_HIGH();

  //jpeg store data

  JpegDec.decodeArray(reinterpret_cast<const uint8_t*>(jpeg_buffer), jpeg_length);

  // Crop the image by keeping a certain number of MCUs in each dimension
  const int keep_x_mcus = width / JpegDec.MCUWidth;
  const int keep_y_mcus = height / JpegDec.MCUHeight;

  // Calculate how many MCUs we will throw away on the x axis
  const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
  // Roughly center the crop by skipping half the throwaway MCUs at the
  // beginning of each row
  const int skip_start_x_mcus = skip_x_mcus / 2;
  // Index where we will start throwing away MCUs after the data
  const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
  // Same approach for the columns
  const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
  const int skip_start_y_mcus = skip_y_mcus / 2;
  const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;
  
  uint16_t *pImg;

  while (JpegDec.read()) {
        // Skip over the initial set of rows
      if (JpegDec.MCUy < skip_start_y_mcus) {
        continue;
      }
      // Skip if we're on a column that we don't want
      if (JpegDec.MCUx < skip_start_x_mcus ||
          JpegDec.MCUx >= skip_end_x_mcu_index) {
        continue;
      }
      // Skip if we've got all the rows we want
      if (JpegDec.MCUy >= skip_end_y_mcu_index) {
        continue;
      }
      // Pointer to the current pixel
      pImg = JpegDec.pImage;

      // The x and y indexes of the current MCU, ignoring the MCUs we skip
      int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
      int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

      // The coordinates of the top left of this MCU when applied to the output
      // image
      int x_origin = relative_mcu_x * JpegDec.MCUWidth;
      int y_origin = relative_mcu_y * JpegDec.MCUHeight;

        // Loop through the MCU's rows and columns
        for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
            // The y coordinate of this pixel in the output index
            int current_y = y_origin + mcu_row;

            for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
                // Read the color of the pixel as 16-bit integer
                uint16_t color = *pImg++;

                // Calculate index
                int current_x = x_origin + mcu_col;
                size_t index = (current_y * width) + current_x;

                // Store the RGB565 pixel to the buffer
                pixel_buffer[index] = color;
            }
        }
    }
    heap_caps_free(jpeg_buffer);
    jpeg_buffer = nullptr;
    delay(2000);
}

// Convert RBG565 pixels into RBG888 pixels
void r565_to_rgb(uint16_t color, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = (color & 0xF800) >> 8;
    *g = (color & 0x07E0) >> 3;
    *b = (color & 0x1F) << 3;
}
// Convert RBG565 pixels into Grayscale pixels
void r565_to_gray(uint16_t color, uint8_t *gray) {
    // Extract the intensity (brightness) from the RGB565 pixel
    int16_t red = ((color & 0xF800)>>11);
    int16_t green = ((color & 0x07E0)>>5);
    int16_t blue = (color & 0x001F);
    int16_t grayscale = (0.2126 * red) + (0.7152 * green ) + (0.0722 * blue);
    *gray=(grayscale<<11)+(grayscale<<6)+grayscale;
    //*gray = ((color & 0xF800) >> 11) + ((color & 0x07E0) >> 5) + ((color & 0x1F));


    // uint8_t r = (color & 0xF800) >> 8;
    // uint8_t g = (color & 0x07E0) >> 3;
    // uint8_t b = (color & 0x1F) << 3;

    // *gray = (r * 0.299) + (g * 0.587) + (b * 0.114);

}
// Data ingestion helper function for grabbing pixels from a framebuffer into Edge Impulse
// This method should be used as the .get_data callback of a signal_t 
int cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    // so offset and length naturally operate on the *cutout*, so we need to cut it out from the real framebuffer
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;
    
    // read byte for byte
    while (bytes_left != 0) {
        // find location of the byte in the cutout
        size_t cutout_row = floor(offset / CUTOUT_COLS);
        size_t cutout_col = offset - (cutout_row * CUTOUT_COLS);

        // then read the value from the real frame buffer
        size_t frame_buffer_row = cutout_row + cutout_row_start;
        size_t frame_buffer_col = cutout_col + cutout_col_start;
        
        uint16_t pixelTemp = pixel_buffer[(frame_buffer_row * FRAME_BUFFER_COLS) + frame_buffer_col];

        uint16_t pixel = (pixelTemp>>8) | (pixelTemp<<8);

        //rgb data
        uint8_t r, g, b;
        r565_to_rgb(pixel, &r, &g, &b);
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;
        // //Calculate the grayscale value from RGB values
        // float pixel_f =  0.299 * r + 0.587 * g + 0.114 *  b;

        //grayscale image
        // uint8_t gray;
        // r565_to_gray(pixel, &gray);
        // out_ptr[out_ptr_ix] = gray;

        // Serial.println(gray);
        // Extracting individual color components
        // int red = (pixel >> 11) & 0x1F;
        // int green = (pixel >> 5) & 0x3F;
        // int blue = pixel & 0x1F;

        // // Normalize color values to the range 0-255
        // red = (red * 255) / 31;
        // green = (green * 255) / 63;
        // blue = (blue * 255) / 31;

        // // Calculate grayscale value
        // //int grayscale = 0.299 * red + 0.587 * green + 0.114 * blue;
        // float grayscale = 0.299 * red + 0.587 * green + 0.114 * blue;
        // out_ptr[out_ptr_ix] = pixel_f;
  
        out_ptr_ix++;
        offset++;
        bytes_left--;
    }
    // and done!
    return 0;
}

//setup 
void setup(){
    uint8_t vid, pid;
  
    pinMode(CS, OUTPUT);
    pinMode(CAM_POWER_ON , OUTPUT);
    digitalWrite(CAM_POWER_ON, HIGH);
    Wire.begin();
    Serial.begin(115200);
    Serial.println(F("ArduCAM Start!"));
    SPI.begin();
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);

    if ((vid != 0x56) || (pid != 0x42)) {
        Serial.println(F("Can't find OV5642 module!"));
    }
    else
        Serial.println(F("OV5642 detected."));
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    delay(1000);
}

//loop
void loop(){
    Serial.println(F("*capture*"));
    size_t size = 320*240;
    pixel_buffer = static_cast<uint16_t*>(heap_caps_malloc(size * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    capture_resize_image( pixel_buffer, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    Serial.println(F("captured and resized"));
    //classifier signal
    signal_t signal;
    signal.total_length = CUTOUT_COLS * CUTOUT_ROWS;
    signal.get_data = &cutout_get_data;
    Serial.println(F("signal get"));
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms. ): \n",result.timing.dsp, result.timing.classification,result.timing.anomaly);
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
          auto bb = result.bounding_boxes[ix];
          if (bb.value == 0) {
              continue;
          }
          ei_printf("    %s (", bb.label);
          ei_printf_float(bb.value);
          ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
      }

      if (!bb_found) {
        ei_printf("    No objects found\n");
        }
    heap_caps_free(pixel_buffer);
    pixel_buffer = nullptr;
    //ESP.deepSleep(5 * 1000000);
} 
