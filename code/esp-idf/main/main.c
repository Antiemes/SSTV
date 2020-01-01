//#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

//#include <esp_http_server.h>
#include "esp_camera.h"

//#include <rom/ets_sys.h>

#include <stddef.h>
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "driver/timer.h"

#include <math.h>
#include <string.h>

#define SERIAL_PIN (GPIO_NUM_14)
#define PTT_PIN (GPIO_NUM_15)

static const char *TAG = "foo";
volatile uint8_t* rgb_buf;
volatile uint16_t rgb_width;
volatile uint16_t rgb_height;

volatile uint8_t serial_buffer[64];

static camera_config_t camera_config = {
  .pin_pwdn = -1,
  .pin_reset = CONFIG_RESET,
  .pin_xclk = CONFIG_XCLK,
  .pin_sscb_sda = CONFIG_SDA,
  .pin_sscb_scl = CONFIG_SCL,

  .pin_d7 = CONFIG_D7,
  .pin_d6 = CONFIG_D6,
  .pin_d5 = CONFIG_D5,
  .pin_d4 = CONFIG_D4,
  .pin_d3 = CONFIG_D3,
  .pin_d2 = CONFIG_D2,
  .pin_d1 = CONFIG_D1,
  .pin_d0 = CONFIG_D0,
  .pin_vsync = CONFIG_VSYNC,
  .pin_href = CONFIG_HREF,
  .pin_pclk = CONFIG_PCLK,

  //XCLK 20MHz or 10MHz
  .xclk_freq_hz = CONFIG_XCLK_FREQ,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_VGA,   //QQVGA-UXGA Do not use sizes above QVGA when not JPEG
  //.frame_size = FRAMESIZE_UXGA,   //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12, //0-63 lower number means higher quality
  .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static esp_err_t init_camera()
{
  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  return ESP_OK;
}

uint16_t lumToFreq(uint8_t lum)
{
  return 1500 + (lum * 3.1372549);
}

static intr_handle_t s_timer_handle;

volatile uint32_t acc=0;
volatile uint32_t freq=1000;

#define SINTABLE_LENGTH 256
uint8_t sintable[SINTABLE_LENGTH];

volatile uint8_t sstv_status=0;
volatile uint16_t sstv_s=0, sstv_y=0;
volatile uint16_t vis_s=0;

static void timer_isr(void* arg)
{
  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[0].config.alarm_en = 1;

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, sintable[acc>>24]);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
  acc+=(freq << 18);

  if (sstv_status==1) //VIS code
  {
    if (vis_s<4687)
    {
      freq=1900;
    }
    else if (vis_s<4844)
    {
      freq=1200;
    }
    else if (vis_s<9531)
    {
      freq=1900;
    }
    else if (vis_s<10000)
    {
      freq=1200;
    }
    else if (vis_s<10469)
    {
      freq=1300;
    }
    else if (vis_s<10937)
    {
      freq=1300;
    }
    else if (vis_s<11406)
    {
      freq=1300;
    }
    else if (vis_s<11875)
    {
      freq=1100;
    }
    else if (vis_s<12344)
    {
      freq=1300;
    }
    else if (vis_s<12812)
    {
      freq=1100;
    }
    else if (vis_s<13281)
    {
      freq=1300;
    }
    else if (vis_s<1750)
    {
      freq=1300;
    }
    else if (vis_s<14218)
    {
      freq=1200;
    }
    else
    {
      sstv_status=2;
    }
    vis_s++;
  }
  else if (sstv_status==2) //SSTV image
  {
    if (sstv_s<76)
    {
      freq=1200;
    }
    else
    {
      uint16_t sstv_x=(sstv_s-76)%1156;
      uint8_t sstv_c=(sstv_s-76)/1156;
      if (sstv_x<9)
      {
        freq=1500;
      }
      else
      {
        uint16_t x=(sstv_x-9)*(rgb_width-1)/1155;
        uint16_t y=sstv_y*(rgb_height-1)/255;
        freq=lumToFreq(rgb_buf[y*rgb_width*3+x*3+sstv_c]);
      }
      //else if (sstv_x<391)
      //{
      //  freq=lumToFreq(0);
      //}
      //else if (sstv_x<774)
      //{
      //  freq=lumToFreq(127);
      //}
      //else
      //{
      //  freq=lumToFreq(255);
      //}
    }

    sstv_s++;
    if (sstv_s>3544)
    {
      sstv_s=0;
      if (sstv_y<255)
      {
        sstv_y++;
      }
      else
      {
        timer_pause(TIMER_GROUP_0, TIMER_0);
        gpio_set_level(PTT_PIN, 1);
        freq=0;
        acc=0;
        sstv_s=0;
        sstv_y=0;
        vis_s=0;
        free(rgb_buf);
        sstv_status=0;
      }
    }
  }
}

void init_timer(int timer_period_us)
{
  timer_config_t config =
  {
    .alarm_en = true,
    .counter_en = false,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = true,
    .divider = 80   /* 1 us per tick */
  };

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);
}

static void timer_isr_serial(void* arg)
{
  static volatile uint8_t bit_counter=0;
  static volatile uint8_t byte_counter=0;
  //timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
  //timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
  TIMERG0.int_clr_timers.t1 = 1;
  TIMERG0.hw_timer[1].config.alarm_en = 1;
  
  if (bit_counter==0)
  {
    gpio_set_level(SERIAL_PIN, 0);
  }
  else if (bit_counter==9)
  {
    gpio_set_level(SERIAL_PIN, 1);
  }
  else
  {
    gpio_set_level(SERIAL_PIN, (serial_buffer[byte_counter] >> (bit_counter-1)) & 1);
  }

  bit_counter++;
  if (bit_counter==10)
  {
    bit_counter=0;
    byte_counter++;
    if (serial_buffer[byte_counter]==0)
    {
      timer_pause(TIMER_GROUP_0, TIMER_1);
      byte_counter=0;
    }
  }
}

void init_timer_serial(int timer_period_us)
{
  timer_config_t config =
  {
    .alarm_en = true,
    .counter_en = false,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = true,
    .divider = 80   /* 1 us per tick */
  };

  timer_init(TIMER_GROUP_0, TIMER_1, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, timer_period_us);
  timer_enable_intr(TIMER_GROUP_0, TIMER_1);
  timer_isr_register(TIMER_GROUP_0, TIMER_1, &timer_isr_serial, NULL, 0, &s_timer_handle);
}

void serial_transmit(char* buf)
{
  strcpy(serial_buffer, buf);
  timer_start(TIMER_GROUP_0, TIMER_1);
}

void sendPic()
{
  gpio_set_level(PTT_PIN, 0);
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  rgb_buf = (uint8_t*)malloc(fb->width*fb->height*3*sizeof(uint8_t));
  rgb_width = fb->width;
  rgb_height = fb->height;
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf);
  esp_camera_fb_return(fb);

  vTaskDelay(1000/portTICK_PERIOD_MS);
  sstv_status=1;
  timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  init_camera();
  
  init_timer(64);

  ledc_timer_config_t timer_conf;
  //timer_conf.bit_num = LEDC_TIMER_15_BIT;
  timer_conf.bit_num = 8; //1-10: numbers, 11-15: enums
  timer_conf.freq_hz = 15625;
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_3;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ledc_conf;
  ledc_conf.channel = LEDC_CHANNEL_1;
  ledc_conf.duty = 50;
  ledc_conf.gpio_num = 2;
  ledc_conf.intr_type = LEDC_INTR_DISABLE;
  ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf.timer_sel = LEDC_TIMER_3;
  ledc_channel_config(&ledc_conf);

  for (int i=0; i<SINTABLE_LENGTH; i++)
  {
    sintable[i]=(sin(i*1.0/SINTABLE_LENGTH*3.14159*2)+1)*127;
  }

  //gpio_set_direction(SERIAL_PIN, GPIO_MODE_OUTPUT);

  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1ULL << SERIAL_PIN) | (1ULL << PTT_PIN);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  gpio_set_level(SERIAL_PIN, 1);
  gpio_set_level(PTT_PIN, 1);
  init_timer_serial(104);
  
  vTaskDelay(1000/portTICK_PERIOD_MS);
  serial_transmit("AT+DMOCONNECT\r\n");
  vTaskDelay(200/portTICK_PERIOD_MS);
  serial_transmit("AT+DMOSETGROUP=0,434.5000,434.5000,0000,6,0000\r\n");
  vTaskDelay(200/portTICK_PERIOD_MS);
  //serial_transmit("COMMAND 2\r\n");
  //vTaskDelay(200/portTICK_PERIOD_MS);

  while(1)
  {
    sendPic();
    vTaskDelay(90*1000/portTICK_PERIOD_MS);
    //ESP_LOGE(TAG, "DELAY vege");
  }

  //ets_delay_us(500);
}
