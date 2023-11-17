#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <geometry_msgs/msg/twist.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <Arduino_GFX_Library.h>
#include <TJpg_Decoder.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#define GFX_BL DF_GFX_BL
#define BACKGROUND BLACK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define READY_LED 16
#define ERROR_LED 13
#define JOYSTICK_X 36
#define JOYSTICK_Y 34
#define JOYSTICK_SW 4

// ROS shared
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ROS subscriber
rcl_subscription_t subscriber;
sensor_msgs__msg__CompressedImage img;

// ROS publisher
rcl_timer_t timer;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist twist;

int jy_dead_zone_low  = 1800;
int jy_dead_zone_high = 1900;
int jy_min = 0;
int jy_max = 4095;

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);

char ssid[] = "";
char psk[]= "";

void error_loop(){
  while(1){
    Serial.println("ERROR!");
    digitalWrite(ERROR_LED, !digitalRead(ERROR_LED));
    delay(100);
  }
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if (y >= 280) return 0;
  gfx->draw16bitRGBBitmap(x, y + 40, bitmap, w, h);
  return 1;
}

void subscription_callback(const void * msgin) { 
  const sensor_msgs__msg__CompressedImage * msg = (const sensor_msgs__msg__CompressedImage *)msgin;
  TJpgDec.drawJpg(0, 0, msg->data.data, msg->data.size);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int x = analogRead(JOYSTICK_X);
    int y = analogRead(JOYSTICK_Y);

    if (x >= jy_dead_zone_low && x <= jy_dead_zone_high) {
      twist.angular.z = 0;
    }
    else if(x < jy_dead_zone_low) {
      twist.angular.z = map(x, jy_min, jy_dead_zone_low, -100.0, 0.0) / 100.0;
    }
    else {
      twist.angular.z = map(x, jy_dead_zone_high, jy_max, 0.0, 100.0) / 100.0;
    }

    if (y >= jy_dead_zone_low && y <= jy_dead_zone_high) {
      twist.linear.x = 0;
    }
    else if(y < jy_dead_zone_low) {
      twist.linear.x = map(y, jy_min, jy_dead_zone_low, -100.0, 0.0) / 100.0;
    }
    else {
      twist.linear.x = map(y, jy_dead_zone_high, jy_max, 0.0, 100.0) / 100.0;
    }

    RCSOFTCHECK(rcl_publish(&publisher, &twist, NULL));
  }
}

void setupGPIO() {
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT);
  pinMode(READY_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(READY_LED, LOW);
  digitalWrite(ERROR_LED, LOW);
}

void setupWIFI() {
  WiFi.begin(ssid, psk);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress agent_ip(192, 168, 0, 79);
  size_t agent_port = 8888;

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
}

void setupTFT() {
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);
  TJpgDec.setCallback(tft_output);

  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }

  gfx->fillScreen(BACKGROUND);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif
}

void setupROS() {
  allocator = rcl_get_default_allocator();

  // Setup node and executor

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Setup subscription

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "webcam_capture_small"));

  img.data.capacity = 6200;
  img.data.size = 0;
  img.data.data = (uint8_t*) malloc(img.data.capacity * sizeof(uint8_t));

  img.header.frame_id.capacity = 10;
  img.header.frame_id.size = 0;
  img.header.frame_id.data = (char*) malloc(img.header.frame_id.capacity * sizeof(char));

  img.header.stamp.nanosec = 0;
  img.header.stamp.sec = 0;

  img.format.capacity = 10;
  img.format.data = (char*) malloc(img.format.capacity * sizeof(char));
  img.format.size = 0;

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &img, &subscription_callback, ON_NEW_DATA));

  // Setup publisher

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
    
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  setupGPIO();
  setupWIFI();
  setupTFT();
  setupROS();

  // Light ready LED
  digitalWrite(READY_LED, HIGH);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
