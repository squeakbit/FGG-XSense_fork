#include "xbox_adapter.h"
#include "log.h"

#ifdef HAS_XBOX_WIRELESS_FIRMWARE
#include "firmware_blob.h"
#endif

#include <sys/ioctl.h>
#include <sys/types.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifdef HAS_XBOX_WIRELESS_FIRMWARE
#define XBOX_ADAPTER_BULK_OUT  0x04
#define XBOX_ADAPTER_BULK_IN_WLAN 0x84
#define XBOX_ADAPTER_BULK_IN_CMD  0x85
#define XBOX_ADAPTER_EP_OUT_INDEX 0
#define XBOX_ADAPTER_EP_IN_CMD_INDEX 1
#define XBOX_ADAPTER_EP_IN_WLAN_INDEX 2
#define XBOX_ADAPTER_INTERFACE 0
#define XBOX_ADAPTER_IN_BUFFER_SIZE 512

#define MT_REQ_DEV_MODE  0x01
#define MT_REQ_WRITE_CFG 0x46
#define MT_REQ_READ_CFG  0x47

#define MT_USB_U3DMA_CFG             0x9018
#define MT_FCE_DMA_ADDR              0x0230
#define MT_FCE_DMA_LEN               0x0234
#define MT_FCE_PSE_CTRL              0x0800
#define MT_TX_CPU_FROM_FCE_BASE_PTR  0x09a0
#define MT_TX_CPU_FROM_FCE_MAX_COUNT 0x09a4
#define MT_TX_CPU_FROM_FCE_DESC_IDX  0x09a8
#define MT_FCE_PDMA_GLOBAL_CONF      0x09c4
#define MT_FCE_SKIP_FS               0x0a6c
#define MT_RF_PATCH                  0x0130

#define MT_FW_LOAD_IVB  0x12
#define MT_MCU_ILM_OFFSET 0x00080000u
#define MT_MCU_DLM_OFFSET 0x00110800u
#define MT_FW_CHUNK_SIZE 0x1000u
#define MT_DMA_COMPLETE 0xc0000000u

#define XFER_TIMEOUT_MS 2000
#define POLL_TIMEOUT_MS 1000
#define POLL_INTERVAL_US 1000
#define USB_FS_INDEX_COUNT 3

typedef struct {
    int fd;
    char operation[128];
    struct usb_fs_endpoint endpoints[USB_FS_INDEX_COUNT];
    void *buffer_pointer[USB_FS_INDEX_COUNT][1];
    uint32_t buffer_length[USB_FS_INDEX_COUNT][1];
    unsigned char buffer[4 + MT_FW_CHUNK_SIZE + 4];
    unsigned char input_buffers[2][XBOX_ADAPTER_IN_BUFFER_SIZE];
    uint16_t endpoint_max_packet_length;
    uint32_t endpoint_max_bufsize;
    int fs_initialized;
    int interface_claimed;
    int input_receives_started;
} adapter_transport;

static void set_operation(adapter_transport *dev, const char *operation)
{
    snprintf(dev->operation, sizeof dev->operation, "%s", operation);
}

static uint32_t read_le32(const unsigned char *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void write_le32(unsigned char *p, uint32_t value)
{
    p[0] = (unsigned char)value;
    p[1] = (unsigned char)(value >> 8);
    p[2] = (unsigned char)(value >> 16);
    p[3] = (unsigned char)(value >> 24);
}

static int get_device_stats(adapter_transport *dev,
                            struct usb_device_stats *stats)
{
    memset(stats, 0, sizeof *stats);
    return ioctl(dev->fd, USB_DEVICESTATS, stats) == 0;
}

static int log_ioctl_failure(adapter_transport *dev, const char *ioctl_name)
{
    int saved_errno = errno;

    log_line("  adapter %s (%s) failed: %s (%d)", ioctl_name,
             dev->operation, strerror(saved_errno), saved_errno);
    errno = saved_errno;
    return 0;
}

static const char *usb_status_name(int status)
{
    switch (status) {
    case 0: return "success";
    case 20: return "timeout";
    case 22: return "stalled";
    default:
        return "unknown";
    }
}

static void probe_device_descriptor(adapter_transport *dev)
{
    struct usb_device_descriptor descriptor;

    memset(&descriptor, 0, sizeof descriptor);
    if (ioctl(dev->fd, USB_GET_DEVICE_DESC, &descriptor) == 0) {
        log_line("  adapter standard USB descriptor probe succeeded: "
                 "VID %04x, PID %04x, USB %04x",
                 UGETW(descriptor.idVendor), UGETW(descriptor.idProduct),
                 UGETW(descriptor.bcdUSB));
    } else {
        int saved_errno = errno;

        log_line("  adapter standard USB descriptor probe failed: %s (%d)",
                 strerror(saved_errno), saved_errno);
    }
}

static int control_transfer(adapter_transport *dev, uint8_t request,
                            uint16_t value, uint16_t index,
                            void *data, uint16_t length, int input)
{
    struct usb_ctl_request control;

    snprintf(dev->operation, sizeof dev->operation,
             "USB request 0x%02x (value 0x%04x, index 0x%04x, %u bytes)",
             request, value, index, length);
    memset(&control, 0, sizeof control);
    control.ucr_data = data;
    control.ucr_request.bmRequestType =
        (uint8_t)(UT_VENDOR | UT_DEVICE | (input ? UT_READ : UT_WRITE));
    control.ucr_request.bRequest = request;
    USETW(control.ucr_request.wValue, value);
    USETW(control.ucr_request.wIndex, index);
    USETW(control.ucr_request.wLength, length);

    if (ioctl(dev->fd, USB_DO_REQUEST, &control) != 0) {
        int saved_errno = errno;

        log_line("  adapter %s failed: %s (%d), bmRequestType 0x%02x, "
                 "actual length %u, request flags 0x%04x",
                 dev->operation, strerror(saved_errno), saved_errno,
                 control.ucr_request.bmRequestType, control.ucr_actlen,
                 control.ucr_flags);
        if (request == MT_REQ_READ_CFG && index == MT_FCE_DMA_ADDR)
            probe_device_descriptor(dev);
        errno = saved_errno;
        return 0;
    }
    if (control.ucr_actlen != length) {
        log_line("  adapter %s returned %u bytes, expected %u",
                 dev->operation, control.ucr_actlen, length);
        errno = EIO;
        return 0;
    }
    return 1;
}

static int control_read(adapter_transport *dev, uint16_t address,
                        uint32_t *value)
{
    unsigned char bytes[4] = { 0, 0, 0, 0 };

    if (!control_transfer(dev, MT_REQ_READ_CFG, 0, address,
                         bytes, sizeof bytes, 1)) return 0;
    *value = read_le32(bytes);
    return 1;
}

static int control_write(adapter_transport *dev, uint16_t address,
                         uint32_t value)
{
    unsigned char bytes[4];

    write_le32(bytes, value);
    return control_transfer(dev, MT_REQ_WRITE_CFG, 0, address,
                            bytes, sizeof bytes, 0);
}

static int device_mode(adapter_transport *dev, uint16_t address)
{
    return control_transfer(dev, MT_REQ_DEV_MODE, address, 0, NULL, 0, 0);
}

static int wait_register(adapter_transport *dev, uint16_t address,
                         uint32_t mask, uint32_t expected)
{
    int waited;
    uint32_t value = 0;

    for (waited = 0; waited < POLL_TIMEOUT_MS; waited++) {
        if (!control_read(dev, address, &value)) return 0;
        if ((value & mask) == expected) return 1;
        usleep(POLL_INTERVAL_US);
    }
    snprintf(dev->operation, sizeof dev->operation,
             "register 0x%04x expected masked value 0x%08x "
             "(mask 0x%08x), last read 0x%08x",
             address, expected, mask, value);
    errno = ETIMEDOUT;
    return 0;
}

static int bulk_write(adapter_transport *dev, const unsigned char *data,
                      uint32_t length)
{
    struct usb_fs_start start;
    struct usb_fs_complete complete;
    struct usb_device_stats stats_before, stats_after;
    int have_stats_before, have_stats_after;
    int waited, polls = 0, completions = 0, input_completions = 0;

    if (!dev->input_receives_started) {
        uint8_t indexes[] = {
            XBOX_ADAPTER_EP_IN_CMD_INDEX,
            XBOX_ADAPTER_EP_IN_WLAN_INDEX
        };
        size_t i;

        for (i = 0; i < sizeof indexes / sizeof indexes[0]; i++) {
            memset(&start, 0, sizeof start);
            start.ep_index = indexes[i];
            set_operation(dev, i == 0 ?
                          "starting USB bulk IN endpoint 0x85" :
                          "starting USB bulk IN endpoint 0x84");
            if (ioctl(dev->fd, USB_FS_START, &start) != 0)
                return log_ioctl_failure(dev, "USB_FS_START (bulk IN)");
        }
        dev->input_receives_started = 1;
        log_line("  adapter bulk IN receives queued on endpoints 0x85 and "
                 "0x84");
    }

    have_stats_before = get_device_stats(dev, &stats_before);
    dev->buffer_pointer[XBOX_ADAPTER_EP_OUT_INDEX][0] = (void *)data;
    dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0] = length;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].nFrames = 1;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].aFrames = 0;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status = 0;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].timeout = XFER_TIMEOUT_MS;

    snprintf(dev->operation, sizeof dev->operation,
             "bulk OUT transfer of %u bytes", length);
    memset(&start, 0, sizeof start);
    start.ep_index = XBOX_ADAPTER_EP_OUT_INDEX;
    if (ioctl(dev->fd, USB_FS_START, &start) != 0)
        return log_ioctl_failure(dev, "USB_FS_START");

    for (waited = 0; waited < XFER_TIMEOUT_MS; waited++) {
        polls++;
        memset(&complete, 0, sizeof complete);
        if (ioctl(dev->fd, USB_FS_COMPLETE, &complete) == 0) {
            completions++;
            if (complete.ep_index != XBOX_ADAPTER_EP_OUT_INDEX) {
                int input_index;

                input_completions++;
                if (complete.ep_index == XBOX_ADAPTER_EP_IN_CMD_INDEX)
                    input_index = 0;
                else if (complete.ep_index == XBOX_ADAPTER_EP_IN_WLAN_INDEX)
                    input_index = 1;
                else
                    continue;

                if (dev->endpoints[complete.ep_index].status == 0 &&
                    dev->endpoints[complete.ep_index].aFrames != 0) {
                    log_line("  adapter USB bulk IN EP 0x%02x received "
                             "%u byte(s)",
                             input_index == 0 ? XBOX_ADAPTER_BULK_IN_CMD :
                                                XBOX_ADAPTER_BULK_IN_WLAN,
                             dev->buffer_length[complete.ep_index][0]);
                } else if (dev->endpoints[complete.ep_index].status != 0) {
                    log_line("  adapter USB bulk IN EP 0x%02x completed "
                             "with status %d (%s)",
                             input_index == 0 ? XBOX_ADAPTER_BULK_IN_CMD :
                                                XBOX_ADAPTER_BULK_IN_WLAN,
                             dev->endpoints[complete.ep_index].status,
                             usb_status_name(
                                 dev->endpoints[complete.ep_index].status));
                }

                dev->buffer_pointer[complete.ep_index][0] =
                    dev->input_buffers[input_index];
                dev->buffer_length[complete.ep_index][0] =
                    sizeof dev->input_buffers[input_index];
                dev->endpoints[complete.ep_index].nFrames = 1;
                dev->endpoints[complete.ep_index].aFrames = 0;
                dev->endpoints[complete.ep_index].status = 0;
                memset(&start, 0, sizeof start);
                start.ep_index = complete.ep_index;
                if (ioctl(dev->fd, USB_FS_START, &start) != 0)
                    return log_ioctl_failure(dev, "USB_FS_START (bulk IN)");
                usleep(POLL_INTERVAL_US);
                continue;
            }
            have_stats_after = get_device_stats(dev, &stats_after);
            if (dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status == 0 &&
                dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].aFrames == 1 &&
                dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0] == length) {
                log_line("  adapter USB bulk OUT completed: %u bytes, "
                         "%d ms, %d poll(s), %d completion(s)",
                         dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0],
                         waited + 1, polls,
                         completions);
                if (have_stats_before && have_stats_after) {
                    log_line("  adapter USB bulk endpoint stats (IN+OUT): "
                             "+ok %u, +fail %u",
                             stats_after.uds_requests_ok[UE_BULK] -
                                 stats_before.uds_requests_ok[UE_BULK],
                             stats_after.uds_requests_fail[UE_BULK] -
                                 stats_before.uds_requests_fail[UE_BULK]);
                }
                return 1;
            }
            log_line("  adapter USB bulk OUT completion: requested %u bytes, "
                     "reported frame length %u, status %d (%s), "
                     "completed frames %u, "
                     "polls %d, completions %d, IN completions %d, "
                     "max packet %u, "
                     "max buffer %u",
                     length,
                     dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0],
                     dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status,
                     usb_status_name(
                         dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status),
                     dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].aFrames,
                     polls, completions,
                     input_completions,
                     dev->endpoint_max_packet_length,
                     dev->endpoint_max_bufsize);
            if (have_stats_before && have_stats_after) {
                log_line("  adapter USB bulk endpoint stats (IN+OUT): "
                         "+ok %u, +fail %u",
                         stats_after.uds_requests_ok[UE_BULK] -
                             stats_before.uds_requests_ok[UE_BULK],
                         stats_after.uds_requests_fail[UE_BULK] -
                             stats_before.uds_requests_fail[UE_BULK]);
            } else {
                log_line("  adapter USB_DEVICESTATS unavailable %s transfer",
                         !have_stats_before ? "before" : "after");
            }
            snprintf(dev->operation, sizeof dev->operation,
                     "bulk OUT transfer failed (USB status %d)",
                     dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status);
            errno = EIO;
            return 0;
        }
        if (errno != EBUSY && errno != EWOULDBLOCK &&
            errno != EAGAIN && errno != EINTR)
            return log_ioctl_failure(dev, "USB_FS_COMPLETE");
        usleep(POLL_INTERVAL_US);
    }

    log_line("  adapter USB_FS_COMPLETE timed out after %d ms; "
             "%d poll(s), %d completion(s), %d IN completion(s), "
             "frame status %d (%s), completed frames %u, "
             "reported frame length %u",
             XFER_TIMEOUT_MS, polls, completions, input_completions,
             dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status,
             usb_status_name(
                 dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].status),
             dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].aFrames,
             dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0]);
    errno = ETIMEDOUT;
    return 0;
}

static void close_transport(adapter_transport *dev)
{
    if (dev->fd < 0) return;

    if (dev->fs_initialized) {
        struct usb_fs_uninit uninit;

        memset(&uninit, 0, sizeof uninit);
        if (ioctl(dev->fd, USB_FS_UNINIT, &uninit) != 0)
            log_ioctl_failure(dev, "USB_FS_UNINIT");
    }
    if (dev->interface_claimed) {
        int iface = XBOX_ADAPTER_INTERFACE;

        if (ioctl(dev->fd, USB_RELEASE_INTERFACE, &iface) != 0)
            log_ioctl_failure(dev, "USB_RELEASE_INTERFACE");
    }
    close(dev->fd);
    dev->fd = -1;
}

static int open_transport(adapter_transport *dev, const char *node)
{
    struct usb_fs_init init;
    struct usb_fs_open endpoint;
    int config;

    memset(dev, 0, sizeof *dev);
    dev->fd = -1;
    set_operation(dev, "opening USB device");
    dev->fd = open(node, O_RDWR);
    if (dev->fd < 0) return 0;

    set_operation(dev, "reading active USB configuration");
    if (ioctl(dev->fd, USB_GET_CONFIG, &config) == 0) {
        log_line("  adapter USB active configuration: %d", config);
    } else {
        int saved_errno = errno;

        log_line("  adapter USB_GET_CONFIG unavailable: %s (%d)",
                 strerror(saved_errno), saved_errno);
    }

    {
        int iface = XBOX_ADAPTER_INTERFACE;

        set_operation(dev, "claiming USB interface");
        if (ioctl(dev->fd, USB_CLAIM_INTERFACE, &iface) != 0)
            return log_ioctl_failure(dev, "USB_CLAIM_INTERFACE");
        dev->interface_claimed = 1;
        log_line("  adapter USB interface 0 claimed");
    }

    dev->buffer_pointer[XBOX_ADAPTER_EP_OUT_INDEX][0] = dev->buffer;
    dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX][0] = sizeof dev->buffer;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].ppBuffer =
        dev->buffer_pointer[XBOX_ADAPTER_EP_OUT_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].pLength =
        dev->buffer_length[XBOX_ADAPTER_EP_OUT_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].nFrames = 1;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].flags =
        USB_FS_FLAG_SINGLE_SHORT_OK;
    dev->endpoints[XBOX_ADAPTER_EP_OUT_INDEX].timeout = XFER_TIMEOUT_MS;

    dev->buffer_pointer[XBOX_ADAPTER_EP_IN_CMD_INDEX][0] =
        dev->input_buffers[0];
    dev->buffer_length[XBOX_ADAPTER_EP_IN_CMD_INDEX][0] =
        sizeof dev->input_buffers[0];
    dev->endpoints[XBOX_ADAPTER_EP_IN_CMD_INDEX].ppBuffer =
        dev->buffer_pointer[XBOX_ADAPTER_EP_IN_CMD_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_IN_CMD_INDEX].pLength =
        dev->buffer_length[XBOX_ADAPTER_EP_IN_CMD_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_IN_CMD_INDEX].nFrames = 1;
    dev->endpoints[XBOX_ADAPTER_EP_IN_CMD_INDEX].flags =
        USB_FS_FLAG_SINGLE_SHORT_OK;
    dev->endpoints[XBOX_ADAPTER_EP_IN_CMD_INDEX].timeout =
        USB_FS_TIMEOUT_NONE;

    dev->buffer_pointer[XBOX_ADAPTER_EP_IN_WLAN_INDEX][0] =
        dev->input_buffers[1];
    dev->buffer_length[XBOX_ADAPTER_EP_IN_WLAN_INDEX][0] =
        sizeof dev->input_buffers[1];
    dev->endpoints[XBOX_ADAPTER_EP_IN_WLAN_INDEX].ppBuffer =
        dev->buffer_pointer[XBOX_ADAPTER_EP_IN_WLAN_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_IN_WLAN_INDEX].pLength =
        dev->buffer_length[XBOX_ADAPTER_EP_IN_WLAN_INDEX];
    dev->endpoints[XBOX_ADAPTER_EP_IN_WLAN_INDEX].nFrames = 1;
    dev->endpoints[XBOX_ADAPTER_EP_IN_WLAN_INDEX].flags =
        USB_FS_FLAG_SINGLE_SHORT_OK;
    dev->endpoints[XBOX_ADAPTER_EP_IN_WLAN_INDEX].timeout =
        USB_FS_TIMEOUT_NONE;

    memset(&init, 0, sizeof init);
    init.pEndpoints = dev->endpoints;
    init.ep_index_max = USB_FS_INDEX_COUNT;
    set_operation(dev, "initializing USB transfer");
    if (ioctl(dev->fd, USB_FS_INIT, &init) != 0)
        return log_ioctl_failure(dev, "USB_FS_INIT");
    dev->fs_initialized = 1;

    memset(&endpoint, 0, sizeof endpoint);
    endpoint.max_bufsize = sizeof dev->buffer;
    endpoint.max_frames = 1;
    endpoint.ep_index = XBOX_ADAPTER_EP_OUT_INDEX;
    endpoint.ep_no = XBOX_ADAPTER_BULK_OUT;
    set_operation(dev, "opening USB bulk endpoint 0x04");
    if (ioctl(dev->fd, USB_FS_OPEN, &endpoint) != 0)
        return log_ioctl_failure(dev, "USB_FS_OPEN");
    dev->endpoint_max_packet_length = endpoint.max_packet_length;
    dev->endpoint_max_bufsize = endpoint.max_bufsize;
    log_line("  adapter USB bulk EP 0x04: max packet %u, max buffer %u",
             dev->endpoint_max_packet_length,
             dev->endpoint_max_bufsize);

    memset(&endpoint, 0, sizeof endpoint);
    endpoint.max_bufsize = sizeof dev->input_buffers[0];
    endpoint.max_frames = 1;
    endpoint.ep_index = XBOX_ADAPTER_EP_IN_CMD_INDEX;
    endpoint.ep_no = XBOX_ADAPTER_BULK_IN_CMD;
    set_operation(dev, "opening USB bulk IN endpoint 0x85");
    if (ioctl(dev->fd, USB_FS_OPEN, &endpoint) != 0)
        return log_ioctl_failure(dev, "USB_FS_OPEN");
    log_line("  adapter USB bulk IN EP 0x85: max packet %u, max buffer %u",
             endpoint.max_packet_length, endpoint.max_bufsize);

    memset(&endpoint, 0, sizeof endpoint);
    endpoint.max_bufsize = sizeof dev->input_buffers[1];
    endpoint.max_frames = 1;
    endpoint.ep_index = XBOX_ADAPTER_EP_IN_WLAN_INDEX;
    endpoint.ep_no = XBOX_ADAPTER_BULK_IN_WLAN;
    set_operation(dev, "opening USB bulk IN endpoint 0x84");
    if (ioctl(dev->fd, USB_FS_OPEN, &endpoint) != 0)
        return log_ioctl_failure(dev, "USB_FS_OPEN");
    log_line("  adapter USB bulk IN EP 0x84: max packet %u, max buffer %u",
             endpoint.max_packet_length, endpoint.max_bufsize);

    return 1;
}

static int send_firmware_chunk(adapter_transport *dev, uint32_t address,
                               const unsigned char *data, uint16_t length)
{
    uint32_t transfer_length = ((uint32_t)length + 3u) & ~3u;
    uint32_t packet_length = 4u + transfer_length + 4u;
    uint32_t dma_address, dma_length;

    dev->buffer[0] = (unsigned char)transfer_length;
    dev->buffer[1] = (unsigned char)(transfer_length >> 8);
    dev->buffer[2] = 0;
    dev->buffer[3] = 0x50;
    memcpy(dev->buffer + 4, data, length);
    memset(dev->buffer + 4 + length, 0,
           packet_length - 4u - length);

    log_line("  adapter firmware chunk: address 0x%08x, firmware bytes %u, "
             "DMA bytes %u, USB packet %u",
             address, length, transfer_length, packet_length);
    if (!control_write(dev, MT_FCE_DMA_ADDR, address) ||
        !control_write(dev, MT_FCE_DMA_LEN, transfer_length << 16))
        return 0;

    if (!control_read(dev, MT_FCE_DMA_ADDR, &dma_address) ||
        !control_read(dev, MT_FCE_DMA_LEN, &dma_length))
        return 0;
    log_line("  adapter DMA registers before bulk OUT: address 0x%08x, "
             "length 0x%08x", dma_address, dma_length);

    if (!bulk_write(dev, dev->buffer, packet_length)) {
        int transfer_errno = errno;
        char transfer_operation[sizeof dev->operation];
        int have_dma_address, have_dma_length;

        memcpy(transfer_operation, dev->operation, sizeof transfer_operation);
        have_dma_address = control_read(dev, MT_FCE_DMA_ADDR, &dma_address);
        have_dma_length = control_read(dev, MT_FCE_DMA_LEN, &dma_length);
        log_line("  adapter DMA registers after failed bulk OUT: "
                 "address %s, length %s",
                 have_dma_address ? "read" : "unavailable",
                 have_dma_length ? "read" : "unavailable");
        if (have_dma_address)
            log_line("    DMA address: 0x%08x", dma_address);
        if (have_dma_length)
            log_line("    DMA length: 0x%08x", dma_length);
        snprintf(dev->operation, sizeof dev->operation, "%s",
                 transfer_operation);
        errno = transfer_errno;
        return 0;
    }

    if (!wait_register(dev, MT_FCE_DMA_LEN, 0xffffffffu,
                       (transfer_length << 16) | MT_DMA_COMPLETE)) {
        log_line("  adapter USB bulk OUT completed but DMA did not: %s",
                 dev->operation);
        return 0;
    }
    log_line("  adapter firmware chunk DMA completed at 0x%08x", address);
    return 1;
}

static int load_firmware(adapter_transport *dev)
{
    const unsigned char *firmware = xbox_wireless_firmware;
    const size_t header_size = 32;
    uint32_t ilm_length, dlm_length, address;
    size_t offset, end;
    uint32_t value;

    if (xbox_wireless_firmware_size < header_size) {
        errno = EINVAL;
        return 0;
    }
    ilm_length = read_le32(firmware);
    dlm_length = read_le32(firmware + 4);
    if ((size_t)ilm_length > xbox_wireless_firmware_size - header_size ||
        (size_t)dlm_length >
            xbox_wireless_firmware_size - header_size - ilm_length) {
        errno = EINVAL;
        return 0;
    }
    log_line("  adapter firmware image: %u bytes, ILM %u bytes, DLM %u bytes, "
             "chunk limit %u",
             (unsigned)xbox_wireless_firmware_size, ilm_length, dlm_length,
             MT_FW_CHUNK_SIZE);

    if (!control_read(dev, MT_FCE_DMA_ADDR, &value)) {
        int saved_errno = errno;
        char failed_operation[sizeof dev->operation];
        uint32_t probe_value;

        snprintf(failed_operation, sizeof failed_operation, "%s",
                 dev->operation);
        log_line("  adapter probing adjacent read-only register 0x%04x",
                 MT_FCE_DMA_LEN);
        if (control_read(dev, MT_FCE_DMA_LEN, &probe_value)) {
            log_line("  adapter adjacent register 0x%04x read succeeded: "
                     "0x%08x", MT_FCE_DMA_LEN, probe_value);
        } else {
            log_line("  adapter adjacent register 0x%04x read failed: %s",
                     MT_FCE_DMA_LEN, strerror(errno));
        }
        snprintf(dev->operation, sizeof dev->operation, "%s",
                 failed_operation);
        errno = saved_errno;
        return 0;
    }
    if (value != 0) {
        if (!control_read(dev, MT_RF_PATCH, &value) ||
            !control_write(dev, MT_RF_PATCH, value & ~(1u << 19)) ||
            !device_mode(dev, MT_FW_LOAD_IVB) ||
            !wait_register(dev, MT_FCE_DMA_ADDR, 0x80000000u,
                           0x80000000u))
            return 0;
    }

    if (!control_write(dev, MT_USB_U3DMA_CFG, 0x00c00000u) ||
        !control_write(dev, MT_FCE_PSE_CTRL, 0x01) ||
        !control_write(dev, MT_TX_CPU_FROM_FCE_BASE_PTR, 0x400230) ||
        !control_write(dev, MT_TX_CPU_FROM_FCE_MAX_COUNT, 0x01) ||
        !control_write(dev, MT_TX_CPU_FROM_FCE_DESC_IDX, 0x01) ||
        !control_write(dev, MT_FCE_PDMA_GLOBAL_CONF, 0x44) ||
        !control_write(dev, MT_FCE_SKIP_FS, 0x03)) return 0;

    offset = header_size;
    end = offset + ilm_length;
    address = MT_MCU_ILM_OFFSET;
    while (offset < end) {
        uint32_t remaining = (uint32_t)(end - offset);
        uint16_t chunk = (uint16_t)(remaining > MT_FW_CHUNK_SIZE ?
                                    MT_FW_CHUNK_SIZE : remaining);

        if (!send_firmware_chunk(dev, address, firmware + offset, chunk))
            return 0;
        offset += chunk;
        address += chunk;
    }

    end = offset + dlm_length;
    address = MT_MCU_DLM_OFFSET;
    while (offset < end) {
        uint32_t remaining = (uint32_t)(end - offset);
        uint16_t chunk = (uint16_t)(remaining > MT_FW_CHUNK_SIZE ?
                                    MT_FW_CHUNK_SIZE : remaining);

        if (!send_firmware_chunk(dev, address, firmware + offset, chunk))
            return 0;
        offset += chunk;
        address += chunk;
    }

    if (!control_write(dev, MT_FCE_DMA_ADDR, 0) ||
        !device_mode(dev, MT_FW_LOAD_IVB) ||
        !wait_register(dev, MT_FCE_DMA_ADDR, 0x01, 0x01)) return 0;

    return 1;
}
#endif

int xbox_adapter_load_firmware(const char *node, char *why, size_t whylen)
{
#ifdef HAS_XBOX_WIRELESS_FIRMWARE
    adapter_transport dev;
    int result = 0;
    int saved_errno;

    if (!open_transport(&dev, node)) {
        saved_errno = errno;
        if (saved_errno == 0) saved_errno = EIO;
        snprintf(why, whylen, "could not configure adapter USB transport at %s: %s",
                 dev.operation, strerror(saved_errno));
        close_transport(&dev);
        return 0;
    }

    if (load_firmware(&dev)) {
        snprintf(why, whylen,
                 "adapter firmware loaded; radio setup and pairing remain "
                 "unsupported");
        result = 1;
    } else {
        saved_errno = errno;
        if (saved_errno == 0) saved_errno = EIO;
        snprintf(why, whylen, "adapter firmware transfer failed at %s: %s",
                 dev.operation, strerror(saved_errno));
    }

    close_transport(&dev);
    return result;
#else
    (void)node;
    snprintf(why, whylen, "adapter firmware is not available in this build");
    return 0;
#endif
}
