/*
 * Copyright (c) 2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "simpleclient.h"
#include <string>
#include <sstream>
#include <vector>
#include "mbed-trace/mbed_trace.h"

#include "security.h"

#include "mbed.h"
#include "rtos.h"

#if(1)  //for zxing with camera
#include "DisplayBace.h"
#include "ImageReaderSource.h"
#endif

#define ETHERNET        1
#define WIFI            2
#define MESH_LOWPAN_ND  3
#define MESH_THREAD     4

#define STRINGIFY(s) #s

#if MBED_CONF_APP_NETWORK_INTERFACE == WIFI
#include "ESP8266Interface.h"
ESP8266Interface esp(MBED_CONF_APP_WIFI_TX, MBED_CONF_APP_WIFI_RX);
#elif MBED_CONF_APP_NETWORK_INTERFACE == ETHERNET
#include "EthernetInterface.h"
EthernetInterface eth;
#elif MBED_CONF_APP_NETWORK_INTERFACE == MESH_LOWPAN_ND
#define MESH
#include "NanostackInterface.h"
LoWPANNDInterface mesh;
#elif MBED_CONF_APP_NETWORK_INTERFACE == MESH_THREAD
#define MESH
#include "NanostackInterface.h"
ThreadInterface mesh;
#endif

#ifndef MESH
// This is address to mbed Device Connector
#define MBED_SERVER_ADDRESS "coap://api.connector.mbed.com:5684"
#else
// This is address to mbed Device Connector
#define MBED_SERVER_ADDRESS "coaps://[2607:f0d0:2601:52::20]:5684"
#endif

Serial output(USBTX, USBRX);

// These are example resource values for the Device Object
struct MbedClientDevice device = {
    "Manufacturer_String",      // Manufacturer
    "Type_String",              // Type
    "ModelNumber_String",       // ModelNumber
    "SerialNumber_String"       // SerialNumber
};


#if(1)  //for zxing with camera
#define VIDEO_CVBS             (0)                 /* Analog  Video Signal */
#define VIDEO_CMOS_CAMERA      (1)                 /* Digital Video Signal */
#define VIDEO_RGB888           (1)

/**** User Selection *********/
/** Camera setting **/
#define VIDEO_INPUT_METHOD     (VIDEO_CMOS_CAMERA) /* Select  VIDEO_CVBS or VIDEO_CMOS_CAMERA                       */
#define VIDEO_INPUT_FORMAT     (VIDEO_RGB888)      /* Select  VIDEO_RGB888                                          */
#define USE_VIDEO_CH           (0)                 /* Select  0 or 1            If selecting VIDEO_CMOS_CAMERA, should be 0.)               */
#define VIDEO_PAL              (0)                 /* Select  0(NTSC) or 1(PAL) If selecting VIDEO_CVBS, this parameter is not referenced.) */
/** LCD setting **/
#define LCD_ONOFF              (1)                 /* Select  0(without LCD) or 1(with LCD) */
#if LCD_ONOFF
#define LCD_TYPE               (1)                 /* Select  0(4.3inch) or 1(7.1inch) */
#endif
/*****************************/

#if LCD_ONOFF
/** LCD shield config **/
#if (LCD_TYPE == 0)
  #include "LCD_shield_config_4_3inch.h"
#else
  #include "LCD_shield_config_7_1inch.h"
#endif
#endif

/** Video and Grapics (GRAPHICS_LAYER_0) parameter **/
/* video input */
#if USE_VIDEO_CH == (0)
  #define VIDEO_INPUT_CH       (DisplayBase::VIDEO_INPUT_CHANNEL_0)
  #define VIDEO_INT_TYPE       (DisplayBase::INT_TYPE_S0_VFIELD)
#else
  #define VIDEO_INPUT_CH       (DisplayBase::VIDEO_INPUT_CHANNEL_1)
  #define VIDEO_INT_TYPE       (DisplayBase::INT_TYPE_S1_VFIELD)
#endif

/* NTSC or PAL */
#if VIDEO_PAL == 0
  #define COL_SYS              (DisplayBase::COL_SYS_NTSC_358)
#else
  #define COL_SYS              (DisplayBase::COL_SYS_PAL_443)
#endif

/* Video input */
#define VIDEO_FORMAT           (DisplayBase::VIDEO_FORMAT_RGB888)
#define GRAPHICS_FORMAT        (DisplayBase::GRAPHICS_FORMAT_RGB888)
#define WR_RD_WRSWA            (DisplayBase::WR_RD_WRSWA_32BIT)

/* The size of the video input */
#if LCD_ONOFF
#if (LCD_TYPE == 0)
  #define VIDEO_PIXEL_HW                LCD_PIXEL_WIDTH
  #define VIDEO_PIXEL_VW                LCD_PIXEL_HEIGHT
#else
  #define VIDEO_PIXEL_HW                (640)  /* VGA */
  #define VIDEO_PIXEL_VW                (480)  /* VGA */
#endif
#else
  #define VIDEO_PIXEL_HW                (640)  /* VGA */
  #define VIDEO_PIXEL_VW                (480)  /* VGA */
#endif

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
/* FRAME BUFFER Parameter GRAPHICS_LAYER_0 */
#define FRAME_BUFFER_NUM              (2u)
#define FRAME_BUFFER_BYTE_PER_PIXEL   (4u)
#define FRAME_BUFFER_STRIDE           (((VIDEO_PIXEL_HW * FRAME_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)
#endif


// Instantiate the class which implements LWM2M Client API (from simpleclient.h)
MbedClient mbed_client(device);


#if(1)  //for zxing with camera
#else
// In case of K64F board , there is button resource available
// to change resource value and unregister
#ifdef TARGET_K64F
// Set up Hardware interrupt button.
InterruptIn obs_button(SW2);
InterruptIn unreg_button(SW3);
#else
//In non K64F boards , set up a timer to simulate updating resource,
// there is no functionality to unregister.
Ticker timer;
#endif
#endif

// LED Output
DigitalOut led1(LED1);

#if(1)  //for zxing with camera
static DisplayBase Display;
#if LCD_ONOFF
static DigitalOut  lcd_pwon(P7_15);
static DigitalOut  lcd_blon(P8_1);
static PwmOut      lcd_cntrst(P8_15);
#endif

/* 32 bytes aligned */
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * VIDEO_PIXEL_VW]__attribute((aligned(32)));
static uint8_t user_frame_buffer1[FRAME_BUFFER_STRIDE * VIDEO_PIXEL_VW]__attribute((aligned(32)));
static uint8_t * decode_buffer = user_frame_buffer0;
static uint8_t * FrameBufferTbl[FRAME_BUFFER_NUM] = {user_frame_buffer0, user_frame_buffer1};
static volatile int32_t lovsync_count = 0;
static volatile int32_t vfield_count = 0;
static int write_buff_num = 0;
static int read_buff_num = 0;
static bool graphics_init_end = false;
#endif


/*
 * The Led contains one property (pattern) and a function (blink).
 * When the function blink is executed, the pattern is read, and the LED
 * will blink based on the pattern.
 */
class LedResource {
public:
    LedResource() {
        // create ObjectID with metadata tag of '3201', which is 'digital output'
        led_object = M2MInterfaceFactory::create_object("3201");
        M2MObjectInstance* led_inst = led_object->create_object_instance();

        // 5853 = Multi-state output
        M2MResource* pattern_res = led_inst->create_dynamic_resource("5853", "Pattern",
            M2MResourceInstance::STRING, false);
        // read and write
        pattern_res->set_operation(M2MBase::GET_PUT_ALLOWED);
        // set initial pattern (toggle every 200ms. 7 toggles in total)
        pattern_res->set_value((const uint8_t*)"500:500:500:500:500:500:500", 27);

        // there's not really an execute LWM2M ID that matches... hmm...
        M2MResource* led_res = led_inst->create_dynamic_resource("5850", "Blink",
            M2MResourceInstance::OPAQUE, false);
        // we allow executing a function here...
        led_res->set_operation(M2MBase::POST_ALLOWED);
        // when a POST comes in, we want to execute the led_execute_callback
        led_res->set_execute_function(execute_callback(this, &LedResource::blink));
    }

    M2MObject* get_object() {
        return led_object;
    }

    void blink(void *) {
        // read the value of 'Pattern'
        M2MObjectInstance* inst = led_object->object_instance();
        M2MResource* res = inst->resource("5853");

        // values in mbed Client are all buffers, and we need a vector of int's
        uint8_t* buffIn = NULL;
        uint32_t sizeIn;
        res->get_value(buffIn, sizeIn);

        // turn the buffer into a string, and initialize a vector<int> on the heap
        std::string s((char*)buffIn, sizeIn);
        std::vector<uint32_t>* v = new std::vector<uint32_t>;

        output.printf("led_execute_callback pattern=%s\r\n", s.c_str());

        // our pattern is something like 500:200:500, so parse that
        std::size_t found = s.find_first_of(":");
        while (found!=std::string::npos) {

            v->push_back(atoi((const char*)s.substr(0,found).c_str()));
            s = s.substr(found+1);
            found=s.find_first_of(":");
            if(found == std::string::npos) {
                v->push_back(atoi((const char*)s.c_str()));
            }
        }


        // do_blink is called with the vector, and starting at -1
        do_blink(v, 0);
    }

private:
    M2MObject* led_object;

    void do_blink(std::vector<uint32_t>* pattern, uint16_t position) {
        // blink the LED
        led1 = !led1;

        // up the position, if we reached the end of the vector
        if (position >= pattern->size()) {
            // free memory, and exit this function
            delete pattern;
            return;
        }

        // how long do we need to wait before the next blink?
        uint32_t delay_ms = pattern->at(position);

        // Invoke same function after `delay_ms` (upping position)
        Thread::wait(delay_ms);
        do_blink(pattern, ++position);
    }
};

/*
 * The button contains one property (click count).
 * When `handle_button_click` is executed, the counter updates.
 */
class ButtonResource {
public:
    ButtonResource(): counter(0) {
        // create ObjectID with metadata tag of '3200', which is 'digital input'
        btn_object = M2MInterfaceFactory::create_object("3200");
        M2MObjectInstance* btn_inst = btn_object->create_object_instance();
        // create resource with ID '5501', which is digital input counter
        M2MResource* btn_res = btn_inst->create_dynamic_resource("5501", "Button",
            M2MResourceInstance::INTEGER, true /* observable */);
        // we can read this value
        btn_res->set_operation(M2MBase::GET_ALLOWED);
        // set initial value (all values in mbed Client are buffers)
        // to be able to read this data easily in the Connector console, we'll use a string
        btn_res->set_value((uint8_t*)"0", 1);        
    }

    ~ButtonResource() {
    }

    M2MObject* get_object() {
        return btn_object;
    }

    /*
     * When you press the button, we read the current value of the click counter
     * from mbed Device Connector, then up the value with one.
     */
#if(1)  //for zxing with camera
    void handle_button_click(char * addr, int size) {
        M2MObjectInstance* inst = btn_object->object_instance();
        M2MResource* res = inst->resource("5501");

        // serialize the value of counter as a string, and tell connector
        res->set_value((uint8_t *)addr, size);
    }
#else
    void handle_button_click() {
        M2MObjectInstance* inst = btn_object->object_instance();
        M2MResource* res = inst->resource("5501");

        // up counter
        counter++;
#ifdef TARGET_K64F
        printf("handle_button_click, new value of counter is %d\r\n", counter);
#else
        printf("simulate button_click, new value of counter is %d\r\n", counter);
#endif
        // serialize the value of counter as a string, and tell connector
        char buffer[20];
        int size = sprintf(buffer,"%d",counter);
        res->set_value((uint8_t*)buffer, size);
    }
#endif

private:
    M2MObject* btn_object;
    uint16_t counter;
};

// Network interaction must be performed outside of interrupt context

#if(1)  //for zxing with camera
volatile bool registered = false;
#else
Semaphore updates(0);
volatile bool registered = false;
volatile bool clicked = false;
#endif
osThreadId mainThread;

#if(1)  //for zxing with camera
// we create our button and LED resources
ButtonResource button_resource;
LedResource led_resource;
#endif

#if(1)  //for zxing with camera
#else
void unregister() {
    registered = false;
    updates.release();
}

void button_clicked() {
    clicked = true;
    updates.release();
}
#endif

// debug printf function
void trace_printer(const char* str) {
    printf("%s\r\n", str);
}

// Status indication
Ticker status_ticker;
DigitalOut status_led(LED1);
void blinky() { status_led = !status_led; }


#if(1)  //for zxing with camera
/****** cache control ******/
static void dcache_clean(void * p_buf, uint32_t size) {
    uint32_t start_addr = (uint32_t)p_buf & 0xFFFFFFE0;
    uint32_t end_addr   = (uint32_t)p_buf + size;
    uint32_t addr;

    /* Data cache clean */
    for (addr = start_addr; addr < end_addr; addr += 0x20) {
        __v7_clean_dcache_mva((void *)addr);
    }
}

#if LCD_ONOFF
/****** LCD ******/
static void IntCallbackFunc_LoVsync(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function for Vsync interruption */
    if (lovsync_count > 0) {
        lovsync_count--;
    }
}

static void Wait_LoVsync(const int32_t wait_count) {
    /* Wait for the specified number of times Vsync occurs */
    lovsync_count = wait_count;
    while (lovsync_count > 0) {
        /* Do nothing */
    }
}

static void Init_LCD_Display(void) {
    DisplayBase::graphics_error_t error;
    DisplayBase::lcd_config_t lcd_config;
    PinName lvds_pin[8] = {
        /* data pin */
        P5_7, P5_6, P5_5, P5_4, P5_3, P5_2, P5_1, P5_0
    };

    lcd_pwon = 0;
    lcd_blon = 0;
    Thread::wait(100);
    lcd_pwon = 1;
    lcd_blon = 1;

    Display.Graphics_Lvds_Port_Init(lvds_pin, 8);

    /* Graphics initialization process */
    lcd_config = LcdCfgTbl_LCD_shield;
    error = Display.Graphics_init(&lcd_config);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
    graphics_init_end = true;

    /* Interrupt callback function setting (Vsync signal output from scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_LO_VSYNC, 0, IntCallbackFunc_LoVsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
}

static void Start_LCD_Display(uint8_t * p_buf) {
    DisplayBase::rect_t rect;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)p_buf,
        FRAME_BUFFER_STRIDE,
        GRAPHICS_FORMAT,
        WR_RD_WRSWA,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);
}
#endif

/****** Video ******/
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function for Vfield interruption */
    if (vfield_count == 0) {
        vfield_count = 1;
    } else {
        vfield_count = 0;
    }
}

static void Wait_Vfield(const int32_t wait_count) {
    /* Wait for the specified number of times Vsync occurs */
    vfield_count = wait_count;
    while (vfield_count > 0) {
        /* Do nothing */
    }
}

static void Init_Video(void) {
    DisplayBase::graphics_error_t error;

    /* Graphics initialization process */
    if (graphics_init_end == false) {
        /* When not initializing LCD, this processing is needed. */
        error = Display.Graphics_init(NULL);
        if (error != DisplayBase::GRAPHICS_OK) {
            printf("Line %d, error %d\n", __LINE__, error);
            mbed_die();
        }
        graphics_init_end = true;
    }

#if VIDEO_INPUT_METHOD == VIDEO_CVBS
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_VDEC, NULL);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#elif VIDEO_INPUT_METHOD == VIDEO_CMOS_CAMERA
    DisplayBase::video_ext_in_config_t ext_in_config;
    PinName cmos_camera_pin[11] = {
        /* data pin */
        P2_7, P2_6, P2_5, P2_4, P2_3, P2_2, P2_1, P2_0,
        /* control pin */
        P10_0,      /* DV0_CLK   */
        P1_0,       /* DV0_Vsync */
        P1_1        /* DV0_Hsync */
    };

    /* MT9V111 camera input config */
    ext_in_config.inp_format     = DisplayBase::VIDEO_EXTIN_FORMAT_BT601; /* BT601 8bit YCbCr format */
    ext_in_config.inp_pxd_edge   = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing data          */
    ext_in_config.inp_vs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Vsync signals */
    ext_in_config.inp_hs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Hsync signals */
    ext_in_config.inp_endian_on  = DisplayBase::OFF;                      /* External input bit endian change on/off       */
    ext_in_config.inp_swap_on    = DisplayBase::OFF;                      /* External input B/R signal swap on/off         */
    ext_in_config.inp_vs_inv     = DisplayBase::SIG_POL_NOT_INVERTED;     /* External input DV_VSYNC inversion control     */
    ext_in_config.inp_hs_inv     = DisplayBase::SIG_POL_INVERTED;         /* External input DV_HSYNC inversion control     */
    ext_in_config.inp_f525_625   = DisplayBase::EXTIN_LINE_525;           /* Number of lines for BT.656 external input */
    ext_in_config.inp_h_pos      = DisplayBase::EXTIN_H_POS_CRYCBY;       /* Y/Cb/Y/Cr data string start timing to Hsync reference */
    ext_in_config.cap_vs_pos     = 6;                                     /* Capture start position from Vsync */
    ext_in_config.cap_hs_pos     = 150;                                   /* Capture start position form Hsync */
#if LCD_ONOFF
#if (LCD_TYPE == 0)
    /* The same screen ratio as the screen ratio of the LCD. */
    ext_in_config.cap_width      = 640;                                   /* Capture width */
    ext_in_config.cap_height     = 363;                                   /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
#else
    ext_in_config.cap_width      = 640;                                   /* Capture width */
    ext_in_config.cap_height     = 468;                                   /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
#endif
#else
    ext_in_config.cap_width      = 640;                                   /* Capture width */
    ext_in_config.cap_height     = 468;                                   /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
#endif
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_EXT, &ext_in_config);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Camera input port setting */
    error = Display.Graphics_Dvinput_Port_Init(cmos_camera_pin, 11);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#endif

    /* Interrupt callback function setting (Field end signal for recording function in scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
}

static void Start_Video(uint8_t * p_buf) {
    DisplayBase::graphics_error_t error;

    /* Video capture setting (progressive form fixed) */
    error = Display.Video_Write_Setting(
                VIDEO_INPUT_CH,
                COL_SYS,
                p_buf,
                FRAME_BUFFER_STRIDE,
                VIDEO_FORMAT,
                WR_RD_WRSWA,
                VIDEO_PIXEL_VW,
                VIDEO_PIXEL_HW
            );
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process start */
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process stop */
    error = Display.Video_Stop(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process start */
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
}
#endif

/****** main ******/
int main() {
#if(1)  //for zxing with camera
    DisplayBase::graphics_error_t error;
    int wk_num, decode_result;
    vector<Ref<Result> > results;
#endif

#ifndef MBEDTLS_ENTROPY_HARDWARE_ALT

#ifdef MBEDTLS_TEST_NULL_ENTROPY
#warning "mbedTLS security feature is disabled. Connection will not be secure !! Implement proper hardware entropy for your selected hardware."

#else

#error "This hardware does not have entropy, endpoint will not register to Connector.\
You need to enable NULL ENTROPY for your application, but if this configuration change is made then no security is offered by mbed TLS.\
Add MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES and MBEDTLS_TEST_NULL_ENTROPY in mbed_app.json macros to register your endpoint."
#endif

#endif
    status_ticker.attach_us(blinky, 250000);

    // Keep track of the main thread
    mainThread = osThreadGetId();

    // Sets the console baud-rate
    output.baud(115200);

    output.printf("Starting mbed Client example...\r\n");

    mbed_trace_init();
    mbed_trace_print_function_set(trace_printer);

    NetworkInterface *network_interface = 0;
    int connect_success = -1;
#if MBED_CONF_APP_NETWORK_INTERFACE == WIFI
    output.printf("\n\rUsing WiFi \r\n");
    output.printf("\n\rConnecting to WiFi..\r\n");
    connect_success = esp.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD);
    network_interface = &esp;
#elif MBED_CONF_APP_NETWORK_INTERFACE == ETHERNET
    output.printf("Using Ethernet\r\n");
    connect_success = eth.connect();
    network_interface = &eth;
#endif
#ifdef MESH
    output.printf("Using Mesh\r\n");
    output.printf("\n\rConnecting to Mesh..\r\n");
    connect_success = mesh.connect();
    network_interface = &mesh;
#endif
    if(connect_success == 0) {
    output.printf("\n\rConnected to Network successfully\r\n");
    } else {
        output.printf("\n\rConnection to Network Failed %d! Exiting application....\r\n", connect_success);
        return 0;
    }
    const char *ip_addr = network_interface->get_ip_address();
    if (ip_addr) {
        output.printf("IP address %s\r\n", ip_addr);
    } else {
        output.printf("No IP address\r\n");
    }

#if(1)  //for zxing with camera
#else
    // we create our button and LED resources
    ButtonResource button_resource;
    LedResource led_resource;

#ifdef TARGET_K64F
    // On press of SW3 button on K64F board, example application
    // will call unregister API towards mbed Device Connector
    //unreg_button.fall(&mbed_client,&MbedClient::test_unregister);
    unreg_button.fall(&unregister);

    // Observation Button (SW2) press will send update of endpoint resource values to connector
    obs_button.fall(&button_clicked);
#else
    // Send update of endpoint resource values to connector every 15 seconds periodically
    timer.attach(&button_clicked, 15.0);
#endif
#endif

    // Create endpoint interface to manage register and unregister
    mbed_client.create_interface(MBED_SERVER_ADDRESS, network_interface);

    // Create Objects of varying types, see simpleclient.h for more details on implementation.
    M2MSecurity* register_object = mbed_client.create_register_object(); // server object specifying connector info
    M2MDevice*   device_object   = mbed_client.create_device_object();   // device resources object

    // Create list of Objects to register
    M2MObjectList object_list;

    // Add objects to list
    object_list.push_back(device_object);
    object_list.push_back(button_resource.get_object());
    object_list.push_back(led_resource.get_object());

    // Set endpoint registration object
    mbed_client.set_register_object(register_object);

    // Register with mbed Device Connector
    mbed_client.test_register(register_object, object_list);
    registered = true;


#if(1)  //for zxing with camera
#if LCD_ONOFF
    /* Initialization of LCD */
    Init_LCD_Display();    /* When using LCD, please call before than Init_Video(). */
#endif
    /* Initialization of Video */
    Init_Video();

    /* Initialization memory */
    for (int i = 0; i < FRAME_BUFFER_NUM; i++) {
        memset(FrameBufferTbl[i], 0, (FRAME_BUFFER_STRIDE * VIDEO_PIXEL_VW));
        dcache_clean(FrameBufferTbl[i], (FRAME_BUFFER_STRIDE * VIDEO_PIXEL_VW));
    }

    /* Start of Video */
    Start_Video(FrameBufferTbl[write_buff_num]);
    /* Wait for first video drawing */
    Wait_Vfield(2);
#if LCD_ONOFF
    /* Start of LCD */
    Start_LCD_Display(FrameBufferTbl[read_buff_num]);
    /* Backlight on */
    Thread::wait(200);
    lcd_cntrst.write(1.0);
#endif
#endif


    while (true) {
#if(1)  //for zxing with camera
        if (registered) {
            if (vfield_count == 0) {
                Wait_Vfield(2);
            } else {
                Wait_Vfield(1);
            }
            wk_num = write_buff_num + 1;
            if (wk_num >= FRAME_BUFFER_NUM) {
                wk_num = 0;
            }
            if (wk_num != read_buff_num) {
                decode_buffer = FrameBufferTbl[read_buff_num];
                read_buff_num  = wk_num;
                write_buff_num = wk_num;
                /* Change video buffer */
                error = Display.Video_Write_Change(VIDEO_INPUT_CH, FrameBufferTbl[write_buff_num], FRAME_BUFFER_STRIDE);
                if (error != DisplayBase::GRAPHICS_OK) {
                    printf("Line %d, error %d\n", __LINE__, error);
                    mbed_die();
                }
                Wait_Vfield(2);
#if LCD_ONOFF
                /* Change LCD buffer */
                Display.Graphics_Read_Change(DisplayBase::GRAPHICS_LAYER_0, (void *)FrameBufferTbl[read_buff_num]);
                Wait_LoVsync(1);
#endif
                /* Decode barcode image */
                decode_result = ex_decode(decode_buffer, (FRAME_BUFFER_STRIDE * VIDEO_PIXEL_VW), VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, &results);
                if (decode_result == 0) {
                    char ** decode_str;
                    int     size;

                    decode_str = (char **)&(results[0]->getText()->getText());
                    printf("%s\n", *decode_str);
                    size = strlen(*decode_str);
                    button_resource.handle_button_click(*decode_str, size);
                    Thread::wait(500);
                } else {
                    Thread::wait(10);
                }
            }
        } else {
            break;
        }
#else
        updates.wait(25000);
        if(registered) {
            if(!clicked) {
                mbed_client.test_update_register();
            }
        }else {
            break;
        }
        if(clicked) {
           clicked = false;
            button_resource.handle_button_click();
        }
#endif
    }

    mbed_client.test_unregister();
    status_ticker.detach();
}

