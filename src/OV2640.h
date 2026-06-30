#ifndef OV2640_H_
#define OV2640_H_

#include <Arduino.h>
#include <pgmspace.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_camera.h"

class OV2640
{
public:
    OV2640(){
        fb = NULL;
    };
    ~OV2640(){
    };
    esp_err_t init(camera_config_t config);
    // Tear down and re-create the driver with the same config, after the
    // sensor DMA has wedged. Drops the stale framebuffer handle so the next
    // run() doesn't return a freed pointer. Caller must ensure no task is
    // inside run()/fb_get when this is called (delete that task first).
    esp_err_t reinit(void);
    void run(void);
    size_t getSize(void);
    uint8_t *getfb(void);
    int getWidth(void);
    int getHeight(void);
    framesize_t getFrameSize(void);
    pixformat_t getPixelFormat(void);

    void setFrameSize(framesize_t size);
    void setPixelFormat(pixformat_t format);

private:
    void runIfNeeded(); // grab a frame if we don't already have one

    camera_config_t _cam_config;

    camera_fb_t *fb;
};

#endif //OV2640_H_
