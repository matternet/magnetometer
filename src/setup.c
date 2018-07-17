#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>

WORKER_THREAD_TAKEOVER_MAIN(lpwork_thread, LOWPRIO)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO, 512)

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 1024)

#define BOARD_PAL_LINE_SPI_SCK PAL_LINE(GPIOB,3)
#define BOARD_PAL_LINE_SPI_MISO PAL_LINE(GPIOB,4)
#define BOARD_PAL_LINE_SPI_MOSI PAL_LINE(GPIOB,5)
#define BOARD_PAL_LINE_SPI_UWB_CS PAL_LINE(GPIOB,0) // NOTE: never drive high by external source
#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOA,11)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOA,12)
#define BOARD_PAL_LINE_GPS_RX PAL_LINE(GPIOA,2)
#define BOARD_PAL_LINE_GPS_TX PAL_LINE(GPIOA,3)
