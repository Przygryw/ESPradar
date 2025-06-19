#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/uart.h>
#include "esp_flash.h"
#include "esp_system.h"
#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <driver/ledc.h>


#define TRIG GPIO_NUM_13
#define ECHO GPIO_NUM_12
#define SERV GPIO_NUM_14
#define pi 3.14159

double distance = 0;
double GPSx = 0;
double GPSy = 0;
bool updated = 0;

bool direction;
uint8_t angle = 0;

void setServo();
void setSR04 ();
void setGPS(const uart_port_t GPS);
void getLocation(const uart_port_t GPS, double *x, double *y);
void pointLocation(double x, double y, double distance, int angle);
void getDistance(double *distance);
void scanArea();
void getLocationDebug();

void scanning()
{
    while(1){
    scanArea(); 
    //getLocation(UART_NUM_2, &GPSx, &GPSy); 
    getDistance(&distance);
    vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void locatingPoint()
{
    while(1)
    {
        pointLocation(GPSx,GPSy,distance,angle);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void app_main(void)
{
    setGPS(UART_NUM_2);
    while(updated != 1) 
    {
        getLocation(UART_NUM_2, &GPSx, &GPSy); 
    }
    setSR04();
    setServo();   
    xTaskCreate(scanning,"Scan", 1024,NULL,1,NULL);
    xTaskCreate(locatingPoint,"Pointpos",4096,NULL,1,NULL);
    
    
}

void getLocationDebug()
{
    int64_t startT = esp_timer_get_time();
    int64_t stopT;
    double distance;
    gpio_set_level(TRIG,0);
    vTaskDelay(pdMS_TO_TICKS(2)); 
    gpio_set_level(TRIG,1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG,0);
    while(gpio_get_level(ECHO) == 0)
    {
        if(esp_timer_get_time() > startT + 25000) 
        {
            stopT = esp_timer_get_time();
            printf("ERROR: n");
            break;
        }
    }
    startT = esp_timer_get_time();
    while(gpio_get_level(ECHO) == 1)
    {
        if(esp_timer_get_time() > startT + 25000)
        {
            stopT = esp_timer_get_time();
            printf("ERROR: n");
            break;
        }
    }
    stopT = esp_timer_get_time();
    distance = (stopT-startT)*0.034/2;
    printf("%lf\n", distance);
}
void setServo()
{

    ledc_timer_config_t SERVO =
    {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .freq_hz = 50,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_2
    };
    ledc_timer_config(&SERVO);
    ledc_channel_config_t SERVconf ={
        .gpio_num = SERV,
        .timer_sel = LEDC_TIMER_2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .hpoint = 0
    };
    ledc_channel_config(&SERVconf);
    ///ledc_set_pin(SERV,LEDC_SPEED_MODE_MAX,)
}

void setSR04 ()
{
    gpio_reset_pin(TRIG);
    gpio_reset_pin(ECHO);
    gpio_set_direction(TRIG,GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO,GPIO_MODE_INPUT);
}

void setGPS(const uart_port_t GPS)
{
    uart_config_t GPSconf = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS,&GPSconf);
    uart_set_pin(GPS,17,16,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(GPS,256,0,10,NULL,0);
}

void getLocation(const uart_port_t GPS, double *x, double *y)
{
    char data[100];
    char buffer;
    uint8_t length = 0;
    do 
    {
        uart_read_bytes(GPS, &buffer,1,10);
    } 
    while(buffer != '$');
    do
    {
        uart_read_bytes(GPS, &buffer,1,10);
        data[length++] = buffer;
    } while (buffer != '\n');
    data[length] = '\0';
    if(strncmp(data,"GPRMC", 5) == 0)
    {
        double posx = 0;
        double posy = 0;
        uint8_t pot = 8;
        updated = 1;
        printf("%s\n", data);

        for(int i = 18; i < 28; i++)
        {
             if(i != 22){
                posy += (data[i]-48)*pow(10,pot);
                pot--;
              }
        }
        posy = posy/10000000;

        pot = 9;
        for(int i = 31; i < 41; i++)
        {
             if(i != 36){
                posx += (data[i]-48)*pow(10,pot);
                pot--;
              }
        }
        posx = posx/10000000;
        //printf("\n%lf N %lf E\n", posy, posx);
        *x = posx;
        *y = posy;
    }
    
}
void pointLocation(double x, double y, double distance, int angle)
{
    double rad = angle*(pi/180);
    /*
    double dx = sqrt(pow(distance,2) - pow((sin(rad)*distance),2));
    double dy = sqrt(pow(distance,2) - pow((cos(rad)*distance),2));
    */
    long double dx = ((distance*sin(rad))*(0.093817*pi))/3600000000;
    long double dy = ((distance*cos(rad))*(0.093817*pi))/3600000000;
    double newX;
    double newY;
    if(angle > 90)
    {
        newX = x-dx;
        newY = y-dy;
    }
    else {
        newX = x+dx;
        newY = y-dy;
    }

    printf("Px:%lf N| Py:%lf E| D:%lf mm| dx:%Lf| dy:%Lf| Cx:%lf| Cy:%lf| kat:%i\n", x, y, distance, dx, dy, newX, newY, angle);
    //printf("%lf N %lf E\n", dx, dy);
    //printf("%lf N %lf E\n", (x+dx), (y+dy));
}
void getDistance(double *distance)
{
    gpio_set_level(TRIG,1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG,0);
    int64_t startT = esp_timer_get_time();
    int64_t stopT;
    while(gpio_get_level(ECHO) == 0)
    {
        if(esp_timer_get_time() > startT + 25000) 
        {
            stopT = esp_timer_get_time();
            break;
        }
    }
    startT = esp_timer_get_time();
    while(gpio_get_level(ECHO) == 1)
    {
        if(esp_timer_get_time() > startT + 25000)
        {
            stopT = esp_timer_get_time();
            break;
        }
    }
    stopT = esp_timer_get_time();
    *distance = ((stopT-startT)*0.34)/2;
}

void scanArea()
{
    if (direction == 0)
    {
        angle+=5;
    }
    else 
    {
        angle-=5;
    }

    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0,1638+angle*27.3);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0);
    if(angle >= 180 || angle <= 0) direction = !direction;
}