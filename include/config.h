
#define ESP32_CAM_IIC_ADDR 0x51
#define ESP32_MOTOR_IIC_ADDR 0x52

//================== 摄像头模块定义 ==================
#ifdef CAT_TOOL_ESP32_CAM
#define DEVICE_ID "CAT_TOOL_ESP32_CAM"

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define IIC_SDA 12
#define IIC_SCL 13
#define IIC_ADDR ESP32_CAM_IIC_ADDR

#endif

//================== 主控模块定义 ==================
#ifdef CAT_TOOL_ESP32_MOTOR
#define DEVICE_ID "CAT_TOOL_ESP32_MOTOR"

#define LEFT_LEDC_NUM 14  //左轮PWM调速通道
#define LEFT_PIN_PWM 16   //左轮PWM调速针脚
#define LEFT_PIN1 17      //左轮控制针脚-1
#define LEFT_PIN2 21      //左轮控制针脚-2

#define RIGHT_LEDC_NUM 15  //右轮PWM调速通道
#define RIGHT_PIN_PWM 5    //右轮PWM调速针脚
#define RIGHT_PIN1 19      //右轮控制针脚-1
#define RIGHT_PIN2 18      //右轮控制针脚-2

#define STEPPER_FOOD_PIN1 13
#define STEPPER_FOOD_PIN2 12
#define STEPPER_FOOD_PIN3 14
#define STEPPER_FOOD_PIN4 27

#define PWM_RESOLUTION 8                     // pwm分辨率
#define PWM_FREQUENCY 1000                   // pwm分辨率
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1)  // pwm分辨率的最大值

#define IIC_SDA 22
#define IIC_SCL 23
#define IIC_ADDR ESP32_MOTOR_IIC_ADDR

#endif
