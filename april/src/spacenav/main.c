#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>

#include "common/getopt.h"
#include "common/timestamp.h"

#include "lcmtypes/gamepad_t.h"

typedef struct {
    int verbose;

    gamepad_t *gp;
    int present;    // {0, 1}
    int values[8];  // [x, y, z, roll, pitch, yaw, right, left]

    pthread_mutex_t mutex;

    char *device;
    char *channel;
    lcm_t *lcm;
} state_t;

volatile int64_t msg_utime = 0;

int fd = -1;

#define TEST_BIT(bit, array) (array[bit / 8] & (1 << (bit % 8)))

static const unsigned char evtype_bits[(EV_MAX + 7) / 8];
static const useconds_t PERIOD_PRESENT_US =  20000;  // 50 Hz
static const useconds_t PERIOD_NOT_PRESENT_US = 250000;  //  4 Hz
static const useconds_t TIMEOUT_PRESENT_US = 1;
static const int AXIS_MAP[6] = {1,0,2,4,3,5};


gamepad_t *values_to_gamepad_t(int values[])
{
    gamepad_t *gp = (gamepad_t*) calloc(1, sizeof(gamepad_t));
    bzero(gp, sizeof(gamepad_t));
    gp->present = 0;
    gp->naxes = 6;
    gp->axes = (double*) calloc(gp->naxes, sizeof(double));
    for (int i = 0; i < gp->naxes; i++)
        gp->axes[AXIS_MAP[i]] = -values[i];
    gp->buttons = ((0x01 & values[6]) << 1) | (0x01 & values[7]);

    return gp;
}

void make_gamepad_t(state_t *state)
{
    gamepad_t *gp = values_to_gamepad_t(state->values);
    gp->present = 1;
    gp->utime = timestamp_now();
    msg_utime = gp->utime;

    // cleanup memory
    gamepad_t *old_gp = state->gp;

    pthread_mutex_lock(&state->mutex);
    state->gp = gp;
    if (old_gp) {
        free(old_gp->axes);
        free(old_gp);
    }
    pthread_mutex_unlock(&state->mutex);
}

void set_led_state(int led_state)
{
    if (fd == -1)
        return;

    struct input_event event;
    bzero(&event, sizeof event);

	event.type = EV_LED;
	event.code = LED_MISC;
	event.value = led_state;

	if(write(fd, &event, sizeof event) == -1) {
		fprintf(stderr, "NFO: Failed to set LED to %s\n", led_state ? "on" : "off");
	}
}

void close_device()
{
    set_led_state(0);
    if (fd > 0)
        close(fd);
    fd = -1;
}

int init_device(state_t *state)
{
    if (fd > 0)
        close_device(fd);

	if ((fd = open(state->device, O_RDWR)) < 0) {
        if ((fd = open(state->device, O_RDONLY)) < 0) {
            perror("WNG: Failed to open device");
            return -1;
        }
        perror("WNG: Device opened in read only mode.");
    }
    /* printf("fd: %d\n", fd); */

    // print name of device
    char name[64]= "Unknown";
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
        perror("WNG: Could not retrieve device name");
    } else
        printf("NFO: Connected to device named '%s'\n", name);

	/* try to grab the device */
	int grab = 1;
	if(ioctl(fd, EVIOCGRAB, &grab) == -1) {
		perror("WRN: failed to grab the spacenav device");
	}

    memset(&evtype_bits, 0, sizeof(evtype_bits));
    if (ioctl(fd, EVIOCGBIT(0, EV_MAX), evtype_bits) < 0) {
        perror("ERR: Failed to get EVIOCGBITS");
        close_device();
        return -1;
    }

    // print available operations
    if (state->verbose) {

        printf("NFO: Supported event types:\n");
        for (int i = 0; i < EV_MAX; i++) {
            if (TEST_BIT(i, evtype_bits)) {
                /* the bit is set in the event types list */
                printf("NFO:    Event type 0x%02x ", i);
                switch (i) {
                    case EV_SYN:
                        printf(" (Synch Events)\n");
                        break;
                    case EV_KEY:
                        printf(" (Keys or Buttons)\n");
                        break;
                    case EV_REL:
                        printf(" (Relative Axes)\n");
                        break;
                    case EV_ABS:
                        printf(" (Absolute Axes)\n");
                        break;
                    case EV_MSC:
                        printf(" (Miscellaneous)\n");
                        break;
                    case EV_LED:
                        printf(" (LEDs)\n");
                        break;
                    case EV_SND:
                        printf(" (Sounds)\n");
                        break;
                    case EV_REP:
                        printf(" (Repeat)\n");
                        break;
                    case EV_FF:
                    case EV_FF_STATUS:
                        printf(" (Force Feedback)\n");
                        break;
                    case EV_PWR:
                        printf(" (Power Management)\n");
                        break;
                    default:
                        printf(" (Unknown: 0x%04hx)\n", i);
                }
            }
        }
    }

    set_led_state(1);
    return 0;
}


int read_device(state_t *state)
{
    if (fd < 0)
        return 1;

	struct input_event in_event;
    int IN_SIZE = sizeof(in_event);

    int ret;
    do {
        ret = read(fd, &in_event, IN_SIZE);
    } while (ret == 0 || (ret == -1 && errno == EINTR));
    state->present = (ret != -1);
    if (ret == -1) {
		if(errno != EAGAIN) {
            perror("Error reading");
			close(fd);
			fd = -1;
		}
        return -1;
    }
    if (state->verbose) {
        printf("DEBUG: read %2d bytes: ", ret);
        for (int i = 0; i < IN_SIZE; i++) {
            printf("%02X ", ((char *)&in_event)[i] & 0xFF);
            if (i % 4 == 3)
                printf("  ");
        }
        printf("\n");
        printf("DEBUG: type: %2d\tcode: %2d\tvalue: %2d\n", in_event.type, in_event.code, in_event.value);
    }

    int axis;
    switch(in_event.type) {
		case EV_REL:
			axis = in_event.code - REL_X;
			state->values[axis] = in_event.value;
			break;
		case EV_ABS:
			axis = in_event.code - ABS_X;
			state->values[axis] = in_event.value;
			break;

		case EV_KEY:
            axis = in_event.code - BTN_0;
			state->values[axis + 6] = in_event.value;
			break;
		case EV_SYN:
            make_gamepad_t(state);
			break;
		default:
            break;
    }
    return 0;
}

static void cleanup()
{
    printf("NFO: Cleaning up\n");
    close_device();
}

static void sig_handler(int s)
{
	switch(s) {
        case SIGSEGV:
        case SIGINT:
        case SIGTERM:
            exit(0);
        default:
            break;
	}
}

void *publish_task(void *arg)
{
    state_t *state = (state_t*)arg;
    gamepad_t *gp_zero = values_to_gamepad_t(state->values);  // zero valued
    int present = 0;
    int64_t utime;

    while (1) {
        if (present)
            usleep(PERIOD_PRESENT_US);
        else
            usleep(PERIOD_NOT_PRESENT_US);

        utime = timestamp_now();
        present = state->present;

        int send_zero = 1;
        if (present) {
            pthread_mutex_lock(&state->mutex);
            if (state->gp) {
                state->gp->utime = utime;
                gamepad_t_publish(state->lcm, state->channel, state->gp);
                send_zero = 0;
            }
            pthread_mutex_unlock(&state->mutex);
        }
        if (send_zero) {
            gp_zero->utime = utime;
            gp_zero->present = present;
            gamepad_t_publish(state->lcm, state->channel, gp_zero);
        }
    }
}

int main(int argc, char *argv[])
{
    state_t *state = (state_t*) calloc(1, sizeof(state_t));

    setlinebuf (stdout);
    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt,   'h', "help",    0,               "Show this");
    getopt_add_bool(gopt,   'v', "verbose", 0,               "Verbose output");
    getopt_add_string(gopt, 'c', "channel", "SPACENAV",      "LCM channel name");
    getopt_add_string(gopt, 'd', "device",  "/dev/spacenav", "Path to device");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help") || varray_size(gopt->extraargs) !=0 ) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        return 0;
    }

    state->lcm = lcm_create(NULL);
    state->channel = getopt_get_string(gopt,"channel");
    state->device = getopt_get_string(gopt,"device");
    state->verbose = getopt_get_bool(gopt,"verbose");

    printf("NFO: Opening device '%s' and publishing on channel '%s'\n", state->device, state->channel);

	atexit(cleanup);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	signal(SIGSEGV, sig_handler);

    pthread_mutex_init(&state->mutex, NULL);
    pthread_t pt;
    pthread_create(&pt, NULL, publish_task, (void *)state);

    init_device(state);
    while (1) {
        if (read_device(state)) {
            usleep(100000);
            init_device(state);
        }
    }
}
