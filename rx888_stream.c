/*

Copyright (c)  2021 Ruslan Migirov <trapi78@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "ezusb.h"
#include <errno.h>
#include <getopt.h>
#include <libusb.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

unsigned int queuedepth = 16; // Number of requests to queue
unsigned int reqsize = 8;     // Request size in number of packets
unsigned int duration = 100;  // Duration of the test in seconds

const char *firmware = NULL;

static unsigned int ep = 1 | LIBUSB_ENDPOINT_IN;

static int interface_number = 0;
static struct libusb_device_handle *dev_handle = NULL;
unsigned int pktsize;
unsigned int success_count = 0;  // Number of successful transfers
unsigned int failure_count = 0;  // Number of failed transfers
unsigned int block_count = 0;   // number of blocks that transfer_callback has seen.  first one is '1'
unsigned int transfer_size = 0;  // Size of data transfers performed so far
unsigned int transfer_index = 0; // Write index into the transfer_size array
volatile bool stop_transfers = false; // Request to stop data transfers
volatile int xfers_in_progress = 0;

// array for histogram of the stream of unint16 samples
unsigned long long sample_histogram[65536] = {0};

volatile int sleep_time = 0;

int verbose;

static int has_firmware;

static uint16_t changedBitsAllBlocks = 0;
static uint16_t initialFirstValue = 0;

static uint16_t findStuckBitsInBlock (uint16_t *samples, size_t sampleCount, uint16_t firstValue) {
    uint16_t changedBits = 0;
    uint16_t thisChangesBits = 0;
    for (int i = 0; i < sampleCount; i++) {
        uint16_t sample = *samples++;
        sample_histogram[sample]++;

        thisChangesBits = firstValue ^ sample;
        changedBits |= thisChangesBits;
    }

    return changedBits;
}

static void explainStuckBits (unsigned int block_count, uint16_t firstValue, uint16_t changedBits) {
    // there are bits stuck on or off
    uint16_t unchangedBits = ~changedBits;
    uint16_t stuckOn = firstValue & unchangedBits;
    uint16_t stuckOff = ~firstValue & unchangedBits;
    if (block_count == 0) {
        fprintf(stderr, "Total Blocks %d bits stuck On: %x, bits stuck Off: %x\n", success_count+failure_count, stuckOn, stuckOff);
    }
    else {
        fprintf(stderr, "Block %u, Error! bits stuck On: %x, bits stuck Off: %x", block_count, stuckOn, stuckOff);
    }
}

static void transfer_callback(struct libusb_transfer *transfer) {
    int size = 0;

    block_count++;
    xfers_in_progress--;

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        failure_count++;
        fprintf(stderr, "Transfer callback status %s received %d bytes.\n",
                libusb_error_name(transfer->status), transfer->actual_length);
    } else {
        size = transfer->actual_length;
        success_count++;
        uint16_t *samples = (uint16_t *)transfer->buffer;

        uint16_t firstValue;
        if (block_count == 1) {
            firstValue = initialFirstValue = *samples;
        }
        else {
            firstValue = initialFirstValue;
        }
        uint16_t changedBits = findStuckBitsInBlock (samples, size / sizeof(uint16_t), firstValue);
        changedBitsAllBlocks |= changedBits;

        if (false) {
            if (changedBits != 0xffff) {
                // there are bits stuck on or off
                explainStuckBits(block_count, firstValue, changedBits);
            }
        }
    }

    if (!stop_transfers) {
        if (libusb_submit_transfer(transfer) == 0)
            xfers_in_progress++;
    }
}

// Function to free data buffers and transfer structures
static void free_transfer_buffers(unsigned char **databuffers,
                                  struct libusb_transfer **transfers) {
    // Free up any allocated data buffers
    if (databuffers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (databuffers[i] != NULL) {
                free(databuffers[i]);
            }
            databuffers[i] = NULL;
        }
        free(databuffers);
    }

    // Free up any allocated transfer structures
    if (transfers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (transfers[i] != NULL) {
                libusb_free_transfer(transfers[i]);
            }
            transfers[i] = NULL;
        }
        free(transfers);
    }
}

static void sig_stop(int signum) {

    (void)signum;
    fprintf(stderr, "\nAbort. Stopping transfers\n");
    stop_transfers = true;
}
static void printhelp(void) {
    fprintf(stderr, " --verbose, -v      Verbose output\n");
    fprintf(stderr, " --firmware, -f     Firmware file\n");
    fprintf(stderr, " --samplerate, -s   Sample Rate, default 32000000\n");
    fprintf(stderr, " --gainmode, -m     Gain Mode low/high, default high\n");
    fprintf(stderr, " --att, -a          Attenuation, default 0\n");
    fprintf(stderr, " --gain, -g         Gain value, default 3\n");
    fprintf(stderr, " --queuedepth, -q   Queue depth, default 16\n");
    fprintf(stderr,
            " --reqsize, -p      Packets per transfer request, default 8\n");
    fprintf(stderr, " --help, -h         Print this help\n");
}
int main(int argc, char **argv) {

    unsigned int samplerate = 129600000;
    unsigned int gain = 0x83;
    unsigned int att = 0;
    int c;
    while (1) {
        static struct option long_options[] = {
            {"verbose", optional_argument, &verbose, 1},
            {"firmware", required_argument, &has_firmware, 'f'},
            {"samplerate", required_argument, 0, 's'},
            {"gainmode", required_argument, 0, 'm'},
            {"gain", required_argument, 0, 'g'},
            {"att", required_argument, 0, 'a'},
            {"queuedepth", required_argument, 0, 'q'},
            {"reqsize", required_argument, 0, 'p'},
            {"help", no_argument, 0, 'h'},
            {0, 0, 0, 0}};

        int option_index = 0;
        int gainvalue = 0;

        c = getopt_long(argc, argv, "f:s:hm:g:a:q:p:", long_options,
                        &option_index);

        if (c == -1)
            break;

        if (c == 0) {
            if (long_options[option_index].flag != 0)
                break;
            c = long_options[option_index].val;
        }
        switch (c) {

        case 'f':
            firmware = optarg;
            break;

        case 'v':
            if (optarg) {
                verbose = strtoul(optarg, NULL, 10);
            } else {
                verbose = 1;
            }
            break;

        case 's':
            samplerate = strtoul(optarg, NULL, 10);
            if (samplerate < 1000000) {
                fprintf(stderr, "Invalid samplerate %d\n", samplerate);
                printhelp();
                return 0;
            }
            break;

        case 'm':
            if (strcmp(optarg, "high") == 0) {
                gain |= 0x80;
            } else if (strcmp(optarg, "low") == 0) {
                gain &= ~0x80;
            } else {
                fprintf(stderr, "Invalid gain mode %s\n", optarg);
                printhelp();
                return 0;
            }
            break;

        case 'g':
            gainvalue = strtol(optarg, NULL, 10);
            if (gainvalue < 0 || gainvalue > 127) {
                fprintf(stderr, "Invalid gain value %d\n", gainvalue);
                printhelp();
                return 0;
            }
            gain &= ~0x7f;
            gain |= gainvalue;
            break;

        case 'a':
            att = strtol(optarg, NULL, 10);
            if (att < 0 || att > 63) {
                fprintf(stderr, "Invalid attenuation value %d\n", att);
                printhelp();
                return 0;
            }
            break;
        case 'q':
            queuedepth = strtol(optarg, NULL, 10);
            if (queuedepth < 1 || queuedepth > 64) {
                fprintf(stderr, "Invalid queue depth %d\n", queuedepth);
                printhelp();
                return 0;
            }
            break;
        case 'p':
            reqsize = strtol(optarg, NULL, 10);
            if (reqsize < 1 || reqsize > 64) {
                fprintf(stderr, "Invalid request size %d\n", reqsize);
                printhelp();
                return 0;
            }
            break;
        case 'h':
        case '?':
        default:
            /* getopt_long already printed an error message. */
            printhelp();
            return 0;
        }
    }

    fprintf(stderr, "Firmware: %s\n", firmware);
    fprintf(stderr, "Sample Rate: %u\n", samplerate);
    fprintf(stderr, "Gain Mode: %s, Gain: %u, Att: %u\n",
            (gain & 0x80) ? "High" : "Low", gain & 0x7f, att);
    /* code */
    struct libusb_device_descriptor desc;
    struct libusb_device *dev;
    struct libusb_endpoint_descriptor const *endpointDesc;
    struct libusb_ss_endpoint_companion_descriptor *ep_comp;
    struct libusb_config_descriptor *config;
    struct libusb_interface_descriptor const *interfaceDesc;
    int ret;
    int rStatus;

    uint16_t vendor_id;  //= 0x04b4;
    uint16_t product_id; // = 0x00f1;

    struct sigaction sigact;

    sigact.sa_handler = sig_stop;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    (void)sigaction(SIGINT, &sigact, NULL);
    (void)sigaction(SIGTERM, &sigact, NULL);

    struct libusb_transfer **transfers = NULL; // List of transfer structures.
    unsigned char **databuffers = NULL;        // List of data buffers.

    ret = libusb_init(NULL);
    if (ret != 0) {
        fprintf(stderr, "Error initializing libusb: %s\n",
                libusb_error_name(ret));
        exit(1);
    }

    if (firmware) { // there is argument with image file
        vendor_id = 0x04b4;
        product_id = 0x00f3;
        // no firmware. upload the firmware
        dev_handle =
            libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
        if (!dev_handle) {
            goto has_firmware;
        }

        dev = libusb_get_device(dev_handle);

        if (ezusb_load_ram(dev_handle, firmware, FX_TYPE_FX3, IMG_TYPE_IMG,
                           1) == 0) {
            fprintf(stderr, "Firmware updated\n");
        } else {
            fprintf(stderr,
                    "Firmware upload failed for "
                    "device %d.%d (logical).\n",
                    libusb_get_bus_number(dev), libusb_get_device_address(dev));
        }

        sleep(2);
    }

has_firmware:

    vendor_id = 0x04b4;
    product_id = 0x00f1;
    dev_handle = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
    if (!dev_handle) {
        fprintf(stderr,
                "Error or device could not be found, try loading firmware\n");
        goto close;
    }

    ret = libusb_kernel_driver_active(dev_handle, 0);
    if (ret != 0) {
        fprintf(stderr,
                "Kernel driver active. Trying to detach kernel driver\n");
        ret = libusb_detach_kernel_driver(dev_handle, 0);
        if (ret != 0) {
            fprintf(stderr,
                    "Could not detach kernel driver from an interface\n");
            goto close;
        }
    }

    dev = libusb_get_device(dev_handle);

    libusb_get_config_descriptor(dev, 0, &config);

    ret = libusb_claim_interface(dev_handle, interface_number);
    if (ret != 0) {
        fprintf(stderr, "Error claiming interface\n");
        goto end;
    }

    fprintf(stderr, "Successfully claimed interface\n");

    interfaceDesc = &(config->interface[0].altsetting[0]);

    endpointDesc = &interfaceDesc->endpoint[0];

    libusb_get_device_descriptor(dev, &desc);

    libusb_get_ss_endpoint_companion_descriptor(NULL, endpointDesc, &ep_comp);

    pktsize = endpointDesc->wMaxPacketSize * (ep_comp->bMaxBurst + 1);

    libusb_free_ss_endpoint_companion_descriptor(ep_comp);

    databuffers = (unsigned char **)calloc(queuedepth, sizeof(u_char *));
    if (databuffers == NULL) {
        fprintf(stderr, "Could not allocate memory for data buffers\n");
        goto end;
    }

    transfers = (struct libusb_transfer **)calloc(
        queuedepth, sizeof(struct libusb_transfer *));
    if (transfers == NULL) {
        fprintf(stderr, "Could not allocate memory for transfer structures\n");
        goto free_transfer_buf;
    }

    fprintf(stderr, "Queue depth: %d, Request size: %d\n", queuedepth,
            reqsize * pktsize);

    for (unsigned int i = 0; i < queuedepth; i++) {
        databuffers[i] = (unsigned char *)malloc(reqsize * pktsize);
        transfers[i] = libusb_alloc_transfer(0);
        if ((databuffers[i] == NULL) || (transfers[i] == NULL)) {
            goto free_transfer_buf;
        }
    }

    for (unsigned int i = 0; i < queuedepth; i++) {
        libusb_fill_bulk_transfer(transfers[i], dev_handle, ep, databuffers[i],
                                  reqsize * pktsize, transfer_callback,
                                  (void *)&pktsize, 0);
        rStatus = libusb_submit_transfer(transfers[i]);
        if (rStatus == 0)
            xfers_in_progress++;
    }

    /******/
    uint32_t gpio = 0;  // 0 is no dither, no randomizer

    usleep(5000);
    command_send(dev_handle, GPIOFX3, gpio);
    usleep(5000);
    argument_send(dev_handle, DAT31_ATT, att);
    usleep(5000);
    argument_send(dev_handle, AD8340_VGA, gain);
    usleep(5000);
    command_send(dev_handle, STARTADC, samplerate);
    usleep(5000);
    command_send(dev_handle, STARTFX3, 0);
    usleep(5000);
    command_send(dev_handle, TUNERSTDBY, 0);
    /*******/

    do {
        libusb_handle_events(NULL);

    } while (stop_transfers != true);

    fprintf(stderr, "Test complete. Stopping transfers\n");
    stop_transfers = true;

    while (xfers_in_progress != 0) {
        fprintf(stderr, "%d transfers are pending\n", xfers_in_progress);
        libusb_handle_events(NULL);
        usleep(100000);
    }

    fprintf(stderr, "Transfers completed\n");

    // here is a good place to report the global stuck bits status
    explainStuckBits(0, initialFirstValue, changedBitsAllBlocks);
    // explain histogram
    fprintf(stderr, "Sample Value Histogram. reporting only missing code points...\n");
    int missingCodes = 0;
    bool inRun = false;
    int runLength = 0;
    int firstInRun = 0;
    int lastInRun = 0;
    for (int i = 0; i < 65536; i++) {
        if (sample_histogram[i] == 0) {
            //fprintf(stderr, "Sample Value 0x%x: count 0\n", i);
            missingCodes++;
            if (!inRun) {
                // start of a run
                firstInRun = i;
                runLength = 0;
                inRun = true;
            }
            lastInRun = i;
            runLength++;
        }
        else {
            if (inRun) {
                if (runLength == 1) {
                    fprintf(stderr, "Sample Value 0x%x missing\n", firstInRun);
                }
                else {
                    fprintf(stderr, "Sample Values from 0x%x to 0x%x missing\n", firstInRun, lastInRun);                   
                }
            }
            inRun = false;
        }
    }
    fprintf(stderr, "There are %d missing Sample Values\n", missingCodes);

    command_send(dev_handle, STOPFX3, 0);

free_transfer_buf:
    free_transfer_buffers(databuffers, transfers);
end:
    if (dev_handle) {
        libusb_release_interface(dev_handle, interface_number);
    }

    if (config) {
        libusb_free_config_descriptor(config);
    }
close:
    if (dev_handle) {
        libusb_close(dev_handle);
    }
    libusb_exit(NULL);

    return 0;
}
