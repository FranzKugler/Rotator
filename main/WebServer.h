#ifdef __cplusplus
extern "C"
{
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_http_server.h"


    struct AngleUpdateData {
        double angle;
        double mechAngle;
        double direction;
    };

    //extern QueueHandle_t angleUpdateQueue;
    void angle_event_broadcast(void*);
    void angle_producer_task(void*);

    void register_web_handles(httpd_handle_t server);

#ifdef __cplusplus
}
#endif