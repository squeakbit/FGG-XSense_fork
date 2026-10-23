#include "xbox_adapter_diag.h"
#include "log.h"

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

#define ADAPTER_INTERFACE 0
#define ADAPTER_BULK_BUFFER 14848
#define ADAPTER_FS_INDEX 0
#define ADAPTER_FS_INDEX_COUNT 1
#define ADAPTER_DESCRIPTOR_BUFFER 1024
#define ADAPTER_REGISTER_ATTEMPTS 5

typedef struct {
    int fd;
    int interface_claimed;
    int fs_initialized;
    int endpoint_open;
    struct usb_fs_endpoint endpoint;
    void *buffer_pointer[1];
    uint32_t buffer_length[1];
    unsigned char buffer[ADAPTER_BULK_BUFFER];
} diagnostic_device;

static int log_ioctl_failure(const char *operation)
{
    int saved_errno = errno;

    log_line("  %s failed: %s (%d)", operation, strerror(saved_errno),
             saved_errno);
    errno = saved_errno;
    return 0;
}

static void dump_bytes(const char *name, const unsigned char *data, size_t len)
{
    size_t offset;

    for (offset = 0; offset < len; offset += 16) {
        char line[16 * 3 + 1];
        size_t i, count = len - offset;
        size_t used = 0;

        if (count > 16) count = 16;
        for (i = 0; i < count; i++) {
            int written = snprintf(line + used, sizeof line - used,
                                   "%s%02x", i == 0 ? "" : " ",
                                   data[offset + i]);
            if (written < 0 || (size_t)written >= sizeof line - used) break;
            used += (size_t)written;
        }
        line[used] = '\0';
        log_line("  %s[%04u]: %s", name, (unsigned)offset, line);
    }
}

static void inspect_configuration(int fd)
{
    unsigned char bytes[ADAPTER_DESCRIPTOR_BUFFER];
    struct usb_gen_descriptor desc;
    size_t offset;

    memset(&desc, 0, sizeof desc);
    desc.ugd_data = bytes;
    desc.ugd_maxlen = sizeof bytes;
    desc.ugd_config_index = 0;
    if (ioctl(fd, USB_GET_FULL_DESC, &desc) != 0) {
        log_ioctl_failure("USB_GET_FULL_DESC");
        return;
    }
    log_line("  configuration descriptor: %u byte(s)", desc.ugd_actlen);
    dump_bytes("config", bytes, desc.ugd_actlen);

    offset = 0;
    while (offset + 2 <= desc.ugd_actlen) {
        unsigned length = bytes[offset];
        unsigned type = bytes[offset + 1];

        if (length < 2 || offset + length > desc.ugd_actlen) {
            log_line("  malformed descriptor at offset %u", (unsigned)offset);
            break;
        }
        if (type == UDESC_INTERFACE && length >= 9) {
            log_line("  interface %u alt %u: class %02x subclass %02x "
                     "protocol %02x",
                     bytes[offset + 2], bytes[offset + 3], bytes[offset + 5],
                     bytes[offset + 6], bytes[offset + 7]);
        } else if (type == UDESC_ENDPOINT && length >= 7) {
            unsigned address = bytes[offset + 2];
            unsigned attributes = bytes[offset + 3];
            unsigned max_packet = (unsigned)bytes[offset + 4] |
                                  ((unsigned)bytes[offset + 5] << 8);

            log_line("  endpoint %02x %s %s max-packet %u interval %u",
                     address, address & UE_DIR_IN ? "IN" : "OUT",
                     (attributes & UE_XFERTYPE) == UE_BULK ? "bulk" :
                     (attributes & UE_XFERTYPE) == UE_INTERRUPT ? "interrupt" :
                     (attributes & UE_XFERTYPE) == UE_ISOCHRONOUS ?
                         "isochronous" : "control",
                     max_packet, bytes[offset + 6]);
        }
        offset += length;
    }
}

static void inspect_port_path(int fd)
{
    struct usb_device_port_path path;
    unsigned i;
    char ports[USB_DEVICE_PORT_PATH_MAX * 4 + 1];
    size_t used = 0;

    memset(&path, 0, sizeof path);
    if (ioctl(fd, USB_GET_DEV_PORT_PATH, &path) != 0) {
        log_ioctl_failure("USB_GET_DEV_PORT_PATH");
        return;
    }
    for (i = 0; i < path.udp_port_level &&
                i < USB_DEVICE_PORT_PATH_MAX; i++) {
        int written = snprintf(ports + used, sizeof ports - used, "%s%u",
                               i == 0 ? "" : ".", path.udp_port_no[i]);
        if (written < 0 || (size_t)written >= sizeof ports - used) break;
        used += (size_t)written;
    }
    ports[used] = '\0';
    log_line("  USB port path: bus %u, device index %u, levels %u, ports %s",
             path.udp_bus, path.udp_index, path.udp_port_level,
             used ? ports : "(none)");
}

static int inspect_register(int fd, uint16_t address, const char *name,
                            int attempts)
{
    int attempt, successes = 0;

    for (attempt = 1; attempt <= attempts; attempt++) {
        unsigned char data[4] = { 0, 0, 0, 0 };
        struct usb_ctl_request request;

        memset(&request, 0, sizeof request);
        request.ucr_data = data;
        request.ucr_request.bmRequestType = UT_VENDOR | UT_DEVICE | UT_READ;
        request.ucr_request.bRequest = 0x47;
        USETW(request.ucr_request.wValue, 0);
        USETW(request.ucr_request.wIndex, address);
        USETW(request.ucr_request.wLength, sizeof data);

        errno = 0;
        {
            int result = ioctl(fd, USB_DO_REQUEST, &request);
            int saved_errno = errno;

            if (result == 0 && request.ucr_actlen == sizeof data)
                successes++;
            log_line("  control-read %s attempt %d: ioctl=%d errno=%d (%s) "
                     "request=%02x/%02x value=%04x index=%04x length=%u "
                     "actlen=%u data=%02x %02x %02x %02x",
                     name, attempt, result, saved_errno,
                     saved_errno ? strerror(saved_errno) : "none",
                     request.ucr_request.bmRequestType,
                     request.ucr_request.bRequest,
                     UGETW(request.ucr_request.wValue),
                     UGETW(request.ucr_request.wIndex),
                     UGETW(request.ucr_request.wLength), request.ucr_actlen,
                     data[0], data[1], data[2], data[3]);
        }
    }
    return successes;
}

static int inspect_registers(int fd, const char *phase, int attempts)
{
    int successes;

    log_line("  register reads %s", phase);
    successes = inspect_register(fd, 0x0230, "DMA_ADDR", attempts);
    successes += inspect_register(fd, 0x0234, "DMA_LEN", attempts);
    log_line("  register probe summary (%s): %d/%d successful reads",
             phase, successes, attempts * 2);
    return successes;
}

static int test_endpoint(diagnostic_device *dev, uint8_t endpoint_number)
{
    struct usb_fs_open open_request;
    struct usb_fs_close close_request;
    int result = 1;

    memset(&open_request, 0, sizeof open_request);
    open_request.max_bufsize = sizeof dev->buffer;
    open_request.max_frames = 1;
    open_request.ep_index = ADAPTER_FS_INDEX;
    open_request.ep_no = endpoint_number;
    log_line("  opening endpoint 0x%02x without starting a transfer",
             endpoint_number);
    if (ioctl(dev->fd, USB_FS_OPEN, &open_request) != 0) {
        log_ioctl_failure("USB_FS_OPEN");
        return 0;
    }
    dev->endpoint_open = 1;
    log_line("  endpoint 0x%02x opened: max packet %u, max buffer %u, "
             "max frames %u",
             endpoint_number, open_request.max_packet_length,
             open_request.max_bufsize, open_request.max_frames);
    inspect_registers(dev->fd, "while endpoint is open", 1);

    memset(&close_request, 0, sizeof close_request);
    close_request.ep_index = ADAPTER_FS_INDEX;
    if (ioctl(dev->fd, USB_FS_CLOSE, &close_request) != 0) {
        log_ioctl_failure("USB_FS_CLOSE");
        result = 0;
    } else {
        dev->endpoint_open = 0;
        log_line("  endpoint 0x%02x closed cleanly", endpoint_number);
        inspect_registers(dev->fd, "after endpoint is closed", 1);
    }
    return result;
}

static void close_diagnostic_device(diagnostic_device *dev)
{
    if (dev->fd < 0) return;

    if (dev->endpoint_open) {
        struct usb_fs_close close_request;

        memset(&close_request, 0, sizeof close_request);
        close_request.ep_index = ADAPTER_FS_INDEX;
        if (ioctl(dev->fd, USB_FS_CLOSE, &close_request) != 0)
            log_ioctl_failure("USB_FS_CLOSE during cleanup");
    }
    if (dev->fs_initialized) {
        struct usb_fs_uninit uninit;

        memset(&uninit, 0, sizeof uninit);
        if (ioctl(dev->fd, USB_FS_UNINIT, &uninit) != 0)
            log_ioctl_failure("USB_FS_UNINIT");
    }
    if (dev->interface_claimed) {
        int iface = ADAPTER_INTERFACE;

        if (ioctl(dev->fd, USB_RELEASE_INTERFACE, &iface) != 0)
            log_ioctl_failure("USB_RELEASE_INTERFACE");
    }
    close(dev->fd);
    dev->fd = -1;
}

int xbox_adapter_run_diagnostics(const char *node, char *why, size_t whylen)
{
    diagnostic_device dev;
    struct usb_device_descriptor device_desc;
    struct usb_device_stats stats_before, stats_after;
    int config = -1, power_usage = -1;
    int passed = 1, have_stats_before, have_stats_after;
    int iface = ADAPTER_INTERFACE;
    int endpoint_result_04, endpoint_result_09;

    memset(&dev, 0, sizeof dev);
    dev.fd = -1;
    log_line("========================================");
    log_line("Xbox Wireless Adapter read-only diagnostic");
    log_line("No interface writes, firmware upload, USB_FS_START, or bulk OUT");
    log_line("  adapter %s", node);

    dev.fd = open(node, O_RDWR);
    if (dev.fd < 0) {
        int saved_errno = errno;

        snprintf(why, whylen, "open %s failed: %s", node,
                 strerror(saved_errno));
        log_line("  %s", why);
        return 0;
    }

    memset(&device_desc, 0, sizeof device_desc);
    if (ioctl(dev.fd, USB_GET_DEVICE_DESC, &device_desc) == 0) {
        dump_bytes("device", (const unsigned char *)&device_desc,
                   sizeof device_desc);
    } else {
        log_ioctl_failure("USB_GET_DEVICE_DESC");
        passed = 0;
    }

    if (ioctl(dev.fd, USB_GET_CONFIG, &config) == 0)
        log_line("  active configuration index: %d", config);
    else {
        log_ioctl_failure("USB_GET_CONFIG");
        passed = 0;
    }

    {
        struct usb_device_info info;

        memset(&info, 0, sizeof info);
        if (ioctl(dev.fd, USB_GET_DEVICEINFO, &info) == 0) {
            log_line("  device info: bus %u address %u index %u speed %u "
                     "config %u suspended %u mode %u",
                     info.udi_bus, info.udi_addr, info.udi_index,
                     info.udi_speed, info.udi_config_no, info.udi_suspended,
                     info.udi_mode);
        } else {
            log_ioctl_failure("USB_GET_DEVICEINFO");
        }
    }

    inspect_port_path(dev.fd);
    if (ioctl(dev.fd, USB_GET_POWER_USAGE, &power_usage) == 0)
        log_line("  USB power usage: %d mA", power_usage);
    else
        log_ioctl_failure("USB_GET_POWER_USAGE");

    inspect_configuration(dev.fd);
    inspect_registers(dev.fd, "before interface claim",
                      ADAPTER_REGISTER_ATTEMPTS);

    have_stats_before = ioctl(dev.fd, USB_DEVICESTATS, &stats_before) == 0;
    if (!have_stats_before) log_ioctl_failure("USB_DEVICESTATS before endpoints");

    if (ioctl(dev.fd, USB_CLAIM_INTERFACE, &iface) != 0) {
        log_ioctl_failure("USB_CLAIM_INTERFACE");
        passed = 0;
    } else {
        dev.interface_claimed = 1;
        log_line("  interface 0 claimed for endpoint setup test");
        inspect_registers(dev.fd, "after interface claim", 1);

        dev.buffer_pointer[0] = dev.buffer;
        dev.buffer_length[0] = sizeof dev.buffer;
        dev.endpoint.ppBuffer = dev.buffer_pointer;
        dev.endpoint.pLength = dev.buffer_length;
        dev.endpoint.nFrames = 1;
        dev.endpoint.flags = USB_FS_FLAG_SINGLE_SHORT_OK;
        dev.endpoint.timeout = 1000;

        {
            struct usb_fs_init init;

            memset(&init, 0, sizeof init);
            init.pEndpoints = &dev.endpoint;
            init.ep_index_max = ADAPTER_FS_INDEX_COUNT;
            if (ioctl(dev.fd, USB_FS_INIT, &init) != 0) {
                log_ioctl_failure("USB_FS_INIT");
                passed = 0;
            } else {
                dev.fs_initialized = 1;
                inspect_registers(dev.fd, "after USB_FS_INIT", 1);
                endpoint_result_04 = test_endpoint(&dev, 0x04);
                endpoint_result_09 = test_endpoint(&dev, 0x09);
                if (!endpoint_result_04 || !endpoint_result_09) passed = 0;
            }
        }
    }

    have_stats_after = ioctl(dev.fd, USB_DEVICESTATS, &stats_after) == 0;
    if (have_stats_before && have_stats_after) {
        log_line("  USB request stats during read-only tests: "
                 "bulk +ok %u +fail %u; control +ok %u +fail %u",
                 stats_after.uds_requests_ok[UE_BULK] -
                     stats_before.uds_requests_ok[UE_BULK],
                 stats_after.uds_requests_fail[UE_BULK] -
                     stats_before.uds_requests_fail[UE_BULK],
                 stats_after.uds_requests_ok[UE_CONTROL] -
                     stats_before.uds_requests_ok[UE_CONTROL],
                 stats_after.uds_requests_fail[UE_CONTROL] -
                     stats_before.uds_requests_fail[UE_CONTROL]);
    } else if (!have_stats_after) {
        log_ioctl_failure("USB_DEVICESTATS after endpoints");
    }

    close_diagnostic_device(&dev);
    snprintf(why, whylen, "read-only diagnostics %s; see xsense.log",
             passed ? "completed" : "completed with errors");
    log_line("  %s", why);
    return passed;
}
