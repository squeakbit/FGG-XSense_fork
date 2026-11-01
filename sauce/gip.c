#include "gip.h"
#include "log.h"
#include "xbox_adapter.h"
#include "xbox_adapter_diag.h"

#include <sys/ioctl.h>
#include <sys/types.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* The interface signature every Xbox-protocol controller presents, and that
 * nothing else does. */
#define GIP_IFACE_CLASS    0xFF
#define GIP_IFACE_SUBCLASS 0x47
#define GIP_IFACE_PROTOCOL 0xD0

/* Microsoft Xbox Wireless Adapter (wireless transport, not a GIP USB pad). */
#define XBOX_WIRELESS_VID 0x045E
#define XBOX_WIRELESS_PID 0x02FE

#define XFER_BUF 256
#define EP_IN    0
#define EP_OUT   1
#define EP_MAX   2

/* Guide LED message payload: which LED, the pattern, and a brightness. */
#define GIP_LED_GUIDE    0x00
#define GIP_LED_GUIDE_ON 0x01
#define GIP_LED_LEVEL    0x14

#define GIP_OPT_CHUNK_START 0x40
#define GIP_OPT_CHUNK       0x80
#define GIP_MAX_PACKET      65535

/* How long gip_init waits for each write to land, in milliseconds. */
#define SEND_TIMEOUT_MS 200

struct gip_chunk_state {
    unsigned char *packet;
    size_t total;
    size_t received;
    unsigned char command;
    unsigned char options;
    unsigned char sequence;
    int active;
};

struct gip_device {
    int  fd;
    char node[64];
    unsigned vendor, product;
    int wireless_adapter;

    int iface, ep_in, ep_out, mps_in;
    int seq;
    int wireless_identified;
    int wireless_initialized;

    struct usb_fs_endpoint eps[EP_MAX];
    void     *bufptr[EP_MAX][1];
    uint32_t  buflen[EP_MAX][1];
    unsigned char data[EP_MAX][XFER_BUF];
    int pending[EP_MAX];

    struct gip_chunk_state chunk;
};

static gip_device g_dev;

struct gip_header {
    unsigned char command;
    unsigned char options;
    unsigned char sequence;
    size_t header_length;
    size_t packet_length;
    size_t chunk_offset;
};

static unsigned word(const unsigned char *p)
{
    return (unsigned)p[0] | ((unsigned)p[1] << 8);
}

static int gip_decode_varint(const unsigned char *msg, size_t length,
                             size_t *cursor, size_t *value)
{
    uint32_t result = 0;
    unsigned shift = 0;
    unsigned count;

    for (count = 0; count < 5; count++) {
        unsigned char byte;

        if (*cursor >= length) return 0;
        byte = msg[(*cursor)++];
        if (count == 4 && (byte & 0xf0u) != 0) return 0;
        result |= (uint32_t)(byte & 0x7fu) << shift;
        if ((byte & 0x80u) == 0) {
            *value = (size_t)result;
            return 1;
        }
        shift += 7;
    }

    return 0;
}

static int gip_parse_header(const unsigned char *msg, size_t length,
                            struct gip_header *header)
{
    size_t cursor = 3;

    if (msg == NULL || header == NULL || length < 4) return 0;

    header->command = msg[0];
    header->options = msg[1];
    header->sequence = msg[2];
    if (!gip_decode_varint(msg, length, &cursor, &header->packet_length))
        return 0;
    header->chunk_offset = 0;
    if ((header->options & GIP_OPT_CHUNK) != 0 &&
        !gip_decode_varint(msg, length, &cursor, &header->chunk_offset))
        return 0;
    if (header->packet_length > length - cursor) return 0;

    header->header_length = cursor;
    return 1;
}

int gip_message_payload(const unsigned char *msg, size_t length,
                        const unsigned char **payload, size_t *payload_length)
{
    struct gip_header header;

    if (payload == NULL || payload_length == NULL ||
        !gip_parse_header(msg, length, &header))
        return 0;

    *payload = msg + header.header_length;
    *payload_length = header.packet_length;
    return 1;
}

static size_t gip_encode_header(unsigned char *msg, unsigned char command,
                                unsigned char options, unsigned char sequence,
                                size_t packet_length)
{
    size_t cursor = 3;
    size_t remaining = packet_length;

    msg[0] = command;
    msg[1] = options;
    msg[2] = sequence;
    do {
        unsigned char byte = (unsigned char)(remaining & 0x7fu);
        remaining >>= 7;
        if (remaining != 0) byte |= 0x80u;
        msg[cursor++] = byte;
    } while (remaining != 0);

    if ((cursor & 1u) != 0) {
        msg[cursor - 1] |= 0x80u;
        msg[cursor++] = 0;
    }

    return cursor;
}

static void gip_clear_chunk(struct gip_chunk_state *chunk)
{
    free(chunk->packet);
    memset(chunk, 0, sizeof *chunk);
}

/* ---- descriptors ------------------------------------------------------- */

/* Walks a configuration descriptor for a GIP interface and the interrupt
 * endpoints belonging to it. Endpoints seen outside that interface are
 * ignored, so a composite device's other functions cannot be mistaken for the
 * gamepad. */
static int scan_layout(gip_device *dev, const unsigned char *buf, int len)
{
    int i = 0, in_gip = 0;

    dev->iface = dev->ep_in = dev->ep_out = -1;
    dev->mps_in = 0;

    while (i + 2 <= len) {
        unsigned blen = buf[i], btype = buf[i + 1];
        if (blen == 0 || i + (int)blen > len) break;

        if (btype == UDESC_INTERFACE && blen >= 9) {
            in_gip = (buf[i + 5] == GIP_IFACE_CLASS &&
                      buf[i + 6] == GIP_IFACE_SUBCLASS &&
                      buf[i + 7] == GIP_IFACE_PROTOCOL);
            if (in_gip && dev->iface < 0) dev->iface = (int)buf[i + 2];
            else in_gip = in_gip && (dev->iface == (int)buf[i + 2]);
        } else if (btype == UDESC_ENDPOINT && blen >= 7 && in_gip) {
            unsigned addr = buf[i + 2], attr = buf[i + 3];
            unsigned mps = (unsigned)buf[i + 4] | ((unsigned)buf[i + 5] << 8);

            if ((attr & UE_XFERTYPE) == UE_INTERRUPT) {
                if ((addr & UE_DIR_IN) && dev->ep_in < 0) {
                    dev->ep_in = (int)addr;
                    dev->mps_in = (int)mps;
                } else if (!(addr & UE_DIR_IN) && dev->ep_out < 0) {
                    dev->ep_out = (int)addr;
                }
            }
        }
        i += (int)blen;
    }

    return dev->iface >= 0 && dev->ep_in >= 0;
}

/* ---- transfers --------------------------------------------------------- */

static int fs_setup(gip_device *dev)
{
    struct usb_fs_init init;
    struct usb_fs_open op;
    int i;

    memset(dev->eps, 0, sizeof dev->eps);
    memset(dev->pending, 0, sizeof dev->pending);

    for (i = 0; i < EP_MAX; i++) {
        dev->bufptr[i][0] = dev->data[i];
        dev->buflen[i][0] = XFER_BUF;
        dev->eps[i].ppBuffer = dev->bufptr[i];
        dev->eps[i].pLength = dev->buflen[i];
        dev->eps[i].nFrames = 1;
        dev->eps[i].flags = USB_FS_FLAG_SINGLE_SHORT_OK;
        dev->eps[i].timeout = 1000;
    }

    memset(&init, 0, sizeof init);
    init.pEndpoints = dev->eps;
    init.ep_index_max = EP_MAX;
    if (ioctl(dev->fd, USB_FS_INIT, &init) != 0) return 0;

    memset(&op, 0, sizeof op);
    op.max_bufsize = XFER_BUF;
    op.max_frames = 1;
    op.ep_index = EP_IN;
    op.ep_no = (uint8_t)dev->ep_in;
    if (ioctl(dev->fd, USB_FS_OPEN, &op) != 0) return 0;

    if (dev->ep_out >= 0) {
        memset(&op, 0, sizeof op);
        op.max_bufsize = XFER_BUF;
        op.max_frames = 1;
        op.ep_index = EP_OUT;
        op.ep_no = (uint8_t)dev->ep_out;
        ioctl(dev->fd, USB_FS_OPEN, &op);   /* input alone is still useful */
    }
    return 1;
}

/* Queues a transfer unless one is already outstanding on that endpoint. */
static int fs_start(gip_device *dev, int index, int length)
{
    struct usb_fs_start start;

    if (dev->pending[index]) return 1;

    dev->buflen[index][0] = (uint32_t)length;
    dev->eps[index].nFrames = 1;
    dev->eps[index].aFrames = 0;
    dev->eps[index].status = 0;

    memset(&start, 0, sizeof start);
    start.ep_index = (uint8_t)index;
    if (ioctl(dev->fd, USB_FS_START, &start) != 0) return 0;

    dev->pending[index] = 1;
    return 1;
}

/* Collects one completion, for whichever endpoint finished.
 *
 * USB_FS_COMPLETE reports a completed endpoint rather than the one the caller
 * is interested in, so the index it returns has to be honoured -- reading the
 * IN endpoint's state after an OUT completion silently produces nonsense.
 */
static int fs_poll(gip_device *dev)
{
    struct usb_fs_complete comp;
    int index;

    memset(&comp, 0, sizeof comp);
    if (ioctl(dev->fd, USB_FS_COMPLETE, &comp) != 0) return 0;

    index = (int)comp.ep_index;
    if (index < 0 || index >= EP_MAX) return 0;

    dev->pending[index] = 0;

    if (index != EP_IN) return 0;
    if (dev->eps[EP_IN].status != 0) return -1;
    if (dev->eps[EP_IN].aFrames == 0) return 0;

    return (int)dev->buflen[EP_IN][0];
}

/* ---- messages ---------------------------------------------------------- */

static int gip_send(gip_device *dev, unsigned char cmd, unsigned char seq,
                    const unsigned char *payload, int plen)
{
    unsigned char *p = dev->data[EP_OUT];

    if (dev->wireless_adapter) {
        unsigned char message[4 + XFER_BUF - 4];

        if (plen < 0 || plen > (int)sizeof message - 4) return 0;
        message[0] = cmd;
        message[1] = GIP_OPT_SYSTEM;
        message[2] = seq;
        message[3] = (unsigned char)plen;
        if (plen > 0) memcpy(message + 4, payload, (size_t)plen);
        return xbox_adapter_send_gip(message, (size_t)(4 + plen));
    }

    if (dev->ep_out < 0) return 0;

    /* Dropping a message whose predecessor is still in flight is safe: every
     * message sent from here is either idempotent or an acknowledgement that
     * will be asked for again. */
    if (dev->pending[EP_OUT]) return 0;

    p[0] = cmd;
    p[1] = GIP_OPT_SYSTEM;
    p[2] = seq;
    p[3] = (unsigned char)plen;
    if (plen > 0) memcpy(p + 4, payload, (size_t)plen);

    return fs_start(dev, EP_OUT, 4 + plen);
}

/* Sends a message and waits for the write itself to complete.
 *
 * The initialisation messages must actually go out rather than be dropped
 * because the previous write is outstanding, so this one waits for the write.
 *
 * Completions arrive on one shared queue, so an input report that lands during
 * the wait is consumed here and lost. That is deliberate and it is cheap: this
 * runs three times per initialisation, reports arrive around ninety times a
 * second, and the alternative is buffering an input report inside the send
 * path to hand back later. Worth knowing it happens, not worth preventing.
 */
static void gip_send_sync(gip_device *dev, unsigned char cmd,
                          const unsigned char *payload, int plen)
{
    struct usb_fs_complete comp;
    int waited = 0;

    if (!gip_send(dev, cmd, (unsigned char)dev->seq++, payload, plen)) return;

    while (dev->pending[EP_OUT] && waited < SEND_TIMEOUT_MS) {
        memset(&comp, 0, sizeof comp);
        if (ioctl(dev->fd, USB_FS_COMPLETE, &comp) == 0) {
            int index = (int)comp.ep_index;
            if (index >= 0 && index < EP_MAX) dev->pending[index] = 0;
        }
        usleep(1000);
        waited++;
    }
}

void gip_init(gip_device *dev)
{
    static const unsigned char power_on[] = { 0x00 };
    static const unsigned char auth[]     = { 0x01, 0x00 };
    static const unsigned char led_on[]   = { GIP_LED_GUIDE, GIP_LED_GUIDE_ON,
                                              GIP_LED_LEVEL };

    if (dev->wireless_adapter) {
        if (!dev->wireless_identified || dev->wireless_initialized) return;
        if (!gip_send(dev, GIP_CMD_SET_DEVICE_STATE,
                      (unsigned char)dev->seq++, power_on,
                      (int)sizeof power_on)) {
            log_line("  failed to send wireless GIP power-on command");
            return;
        }
        if (!gip_send(dev, GIP_CMD_LED, (unsigned char)dev->seq++,
                      led_on, (int)sizeof led_on)) {
            log_line("  failed to send wireless GIP LED command");
            return;
        }
        dev->wireless_initialized = 1;
        log_line("  wireless GIP power-on and LED initialization sent");
        return;
    }
    gip_send_sync(dev, GIP_CMD_SET_DEVICE_STATE, power_on, (int)sizeof power_on);
    gip_send_sync(dev, GIP_CMD_AUTHENTICATE, auth, (int)sizeof auth);
    gip_send_sync(dev, GIP_CMD_LED, led_on, (int)sizeof led_on);
}

void gip_acknowledge(gip_device *dev, const unsigned char *msg)
{
    unsigned char body[9];

    /* The payload carries the command, options and length of the message being
     * answered; the header echoes its sequence number. Mirrors the packet
     * Linux xpad sends in xpadone_ack_mode_report. */
    memset(body, 0, sizeof body);
    body[1] = msg[0];
    body[2] = GIP_OPT_SYSTEM;
    body[3] = msg[3];

    gip_send(dev, GIP_CMD_ACK, msg[2], body, (int)sizeof body);
}

static int gip_acknowledge_chunk(gip_device *dev,
                                 const struct gip_header *header,
                                 size_t offset, size_t total)
{
    unsigned char body[9];
    size_t acknowledged = offset + header->packet_length;

    if (acknowledged > total || acknowledged > 0xffffu) return 0;

    memset(body, 0, sizeof body);
    body[1] = header->command;
    body[2] = GIP_OPT_SYSTEM;
    body[3] = (unsigned char)(acknowledged & 0xffu);
    body[4] = (unsigned char)((acknowledged >> 8) & 0xffu);
    body[7] = (unsigned char)((total - acknowledged) & 0xffu);
    body[8] = (unsigned char)(((total - acknowledged) >> 8) & 0xffu);

    return gip_send(dev, GIP_CMD_ACK, header->sequence, body, (int)sizeof body);
}

static int gip_process_chunk(gip_device *dev, const unsigned char *msg,
                             int length, const unsigned char **complete_msg)
{
    struct gip_header header;
    struct gip_chunk_state *chunk = &dev->chunk;
    size_t offset, total;

    if (!gip_parse_header(msg, (size_t)length, &header) ||
        (header.options & GIP_OPT_CHUNK) == 0) {
        log_line("  invalid wireless GIP chunk header");
        gip_clear_chunk(chunk);
        return -1;
    }

    if ((header.options & GIP_OPT_CHUNK_START) != 0) {
        total = header.chunk_offset;
        if (total == 0 || total > GIP_MAX_PACKET ||
            header.packet_length > total) {
            log_line("  invalid wireless GIP chunk size %u",
                     (unsigned)total);
            gip_clear_chunk(chunk);
            return -1;
        }

        if (chunk->active && chunk->packet != NULL &&
            chunk->command == header.command &&
            chunk->sequence == header.sequence &&
            chunk->total == total &&
            chunk->received >= header.packet_length &&
            memcmp(chunk->packet + 6, msg + header.header_length,
                   header.packet_length) == 0) {
            if ((header.options & GIP_OPT_ACK) != 0 &&
                !gip_acknowledge_chunk(dev, &header, 0, total)) {
                log_line("  failed to acknowledge duplicate wireless GIP "
                         "start chunk");
                return -1;
            }
            return 0;
        }

        gip_clear_chunk(chunk);
        chunk->packet = malloc(total + 6);
        if (chunk->packet == NULL) {
            log_line("  could not allocate %u bytes for wireless GIP chunks",
                     (unsigned)(total + 6));
            return -1;
        }
        chunk->active = 1;
        chunk->total = total;
        chunk->command = header.command;
        chunk->options = header.options;
        chunk->sequence = header.sequence;
        offset = 0;
    } else {
        if (!chunk->active || chunk->packet == NULL ||
            chunk->command != header.command ||
            chunk->sequence != header.sequence) {
            log_line("  wireless GIP chunk arrived without a matching start");
            return -1;
        }
        total = chunk->total;
        offset = header.chunk_offset;
    }

    if (offset > total || header.packet_length > total - offset) {
        log_line("  wireless GIP chunk exceeds message bounds "
                 "(offset %u, length %u, total %u)",
                 (unsigned)offset, (unsigned)header.packet_length,
                 (unsigned)total);
        return -1;
    }

    if (offset < chunk->received) {
        if (header.packet_length <= chunk->received - offset &&
            memcmp(chunk->packet + 6 + offset,
                   msg + header.header_length,
                   header.packet_length) == 0) {
            if ((header.options & GIP_OPT_ACK) != 0 &&
                !gip_acknowledge_chunk(dev, &header, offset, total)) {
                log_line("  failed to acknowledge duplicate wireless GIP "
                         "chunk");
                return -1;
            }
            return 0;
        }
        log_line("  conflicting duplicate wireless GIP chunk at %u",
                 (unsigned)offset);
        return -1;
    }

    if (offset != chunk->received) {
        log_line("  out-of-order wireless GIP chunk at %u (expected %u)",
                 (unsigned)offset, (unsigned)chunk->received);
        return 0;
    }

    if ((header.options & GIP_OPT_ACK) != 0 &&
        !gip_acknowledge_chunk(dev, &header, offset, total)) {
        log_line("  failed to acknowledge wireless GIP chunk");
        gip_clear_chunk(chunk);
        return -1;
    }

    if (header.packet_length > 0) {
        memcpy(chunk->packet + 6 + offset,
               msg + header.header_length, header.packet_length);
        chunk->received += header.packet_length;
        return 0;
    }

    if (chunk->received != total) {
        log_line("  incomplete wireless GIP message: received %u of %u bytes",
                 (unsigned)chunk->received, (unsigned)total);
        gip_clear_chunk(chunk);
        return -1;
    }

    {
        size_t header_length;
        unsigned char encoded_header[6];
        unsigned char options = chunk->options &
            (unsigned char)~(GIP_OPT_ACK | GIP_OPT_CHUNK_START | GIP_OPT_CHUNK);

        header_length = gip_encode_header(encoded_header, chunk->command,
                                          options, chunk->sequence, total);
        if (header_length > 6) {
            log_line("  wireless GIP message header exceeded its buffer");
            gip_clear_chunk(chunk);
            return -1;
        }
        memmove(chunk->packet + header_length, chunk->packet + 6, total);
        memcpy(chunk->packet, encoded_header, header_length);
        *complete_msg = chunk->packet;
        chunk->active = 0;
        log_line("  wireless GIP message reassembled: command 0x%02x, "
                 "%u payload byte(s)",
                 chunk->command, (unsigned)total);
        return (int)(header_length + total);
    }
}

int gip_read(gip_device *dev, const unsigned char **msg)
{
    struct gip_header header;
    int n;

    if (dev->wireless_adapter) {
        n = xbox_adapter_read(msg);
    } else {
        fs_start(dev, EP_IN, dev->mps_in ? dev->mps_in : 64);
        n = fs_poll(dev);
        if (n > 0) *msg = dev->data[EP_IN];
    }

    if (n <= 0) return n;
    if (!gip_parse_header(*msg, (size_t)n, &header)) {
        log_line("  malformed GIP message received (%u bytes)", (unsigned)n);
        return -1;
    }
    if ((header.options & GIP_OPT_CHUNK) != 0) {
        n = gip_process_chunk(dev, *msg, n, msg);
        if (n > 0 && (*msg)[0] == GIP_CMD_IDENTIFY &&
            dev->wireless_adapter) {
            dev->wireless_identified = 1;
            gip_init(dev);
        }
        return n;
    }

    if (header.command == GIP_CMD_IDENTIFY && dev->wireless_adapter) {
        dev->wireless_identified = 1;
        gip_init(dev);
    }

    return n;
}

/* ---- lifecycle --------------------------------------------------------- */

/* Configures, claims and initialises an already-open device node. */
static int bring_up(gip_device *dev)
{
    int cfg = 0;

    ioctl(dev->fd, USB_SET_CONFIG, &cfg);
    ioctl(dev->fd, USB_CLAIM_INTERFACE, &dev->iface);

    if (!fs_setup(dev)) return 0;

    gip_init(dev);
    return 1;
}

static int read_descriptors(int fd, unsigned char *cfg, int cfglen, int *actlen)
{
    struct usb_gen_descriptor gd;

    memset(&gd, 0, sizeof gd);
    gd.ugd_data = cfg;
    gd.ugd_maxlen = (uint16_t)cfglen;
    gd.ugd_config_index = 0;

    if (ioctl(fd, USB_GET_FULL_DESC, &gd) != 0 || gd.ugd_actlen == 0) return 0;
    *actlen = (int)gd.ugd_actlen;
    return 1;
}

static int is_gip(const unsigned char *cfg, int len)
{
    int i = 0;
    while (i + 9 <= len) {
        unsigned blen = cfg[i];
        if (blen == 0) break;
        if (cfg[i + 1] == UDESC_INTERFACE &&
            cfg[i + 5] == GIP_IFACE_CLASS &&
            cfg[i + 6] == GIP_IFACE_SUBCLASS &&
            cfg[i + 7] == GIP_IFACE_PROTOCOL) return 1;
        i += (int)blen;
    }
    return 0;
}

static void log_interfaces(const unsigned char *cfg, int len)
{
    int i = 0, in_interface = 0;

    while (i + 9 <= len) {
        unsigned blen = cfg[i];
        if (blen == 0 || i + (int)blen > len) break;
        if (cfg[i + 1] == UDESC_INTERFACE && blen >= 9) {
            in_interface = 1;
            log_line("    interface %u alt %u: %02X:%02X:%02X",
                     cfg[i + 2], cfg[i + 3], cfg[i + 5], cfg[i + 6], cfg[i + 7]);
        } else if (cfg[i + 1] == UDESC_ENDPOINT && blen >= 7 && in_interface) {
            unsigned address = cfg[i + 2];
            unsigned type = cfg[i + 3] & UE_XFERTYPE;
            unsigned mps = (unsigned)cfg[i + 4] | ((unsigned)cfg[i + 5] << 8);
            const char *type_name = type == UE_BULK ? "bulk" :
                                    type == UE_INTERRUPT ? "interrupt" :
                                    type == UE_ISOCHRONOUS ? "isochronous" : "control";

            log_line("      endpoint %02X %s %s max-packet %u",
                     address, address & UE_DIR_IN ? "IN" : "OUT", type_name, mps);
        }
        i += (int)blen;
    }
}

gip_device *gip_open(char *why, int whylen)
{
    gip_device *dev = &g_dev;
    unsigned char cfg[1024];
    char wireless_node[64] = "";
    int bus, unit, found = 0, cfg_len = 0;

    memset(dev, 0, sizeof *dev);
    dev->fd = -1;

    for (bus = 0; bus < 8 && !found; bus++) {
        for (unit = 1; unit < 16 && !found; unit++) {
            struct usb_device_descriptor dd;
            int fd;

            snprintf(dev->node, sizeof dev->node, "/dev/ugen%d.%d", bus, unit);
            if (access(dev->node, F_OK) != 0) continue;

            fd = open(dev->node, O_RDWR);
            if (fd < 0) continue;

            memset(&dd, 0, sizeof dd);
            ioctl(fd, USB_GET_DEVICE_DESC, &dd);
            dev->vendor = word((const unsigned char *)&dd.idVendor);
            dev->product = word((const unsigned char *)&dd.idProduct);
            log_line("  USB %s %04X:%04X", dev->node, dev->vendor, dev->product);
            if (dev->vendor == XBOX_WIRELESS_VID &&
                dev->product == XBOX_WIRELESS_PID) {
                snprintf(wireless_node, sizeof wireless_node, "%s", dev->node);
            }

            if (!read_descriptors(fd, cfg, (int)sizeof cfg, &cfg_len)) {
                log_line("  USB %s: cannot read configuration descriptors: %s",
                         dev->node, strerror(errno));
                close(fd);
                continue;
            }

            log_interfaces(cfg, cfg_len);

            if (!is_gip(cfg, cfg_len)) {
                close(fd);
                continue;
            }

            dev->fd = fd;
            found = 1;
        }
    }

    if (!found) {
        if (wireless_node[0] != '\0') {
            char adapter_why[256];

#ifdef XBOX_ADAPTER_READ_ONLY_DIAGNOSTIC
            log_line("  Xbox Wireless Adapter at %s: read-only diagnostics",
                     wireless_node);
            xbox_adapter_run_diagnostics(wireless_node, adapter_why,
                                         sizeof adapter_why);
#else
            log_line("  Xbox Wireless Adapter at %s: loading firmware",
                     wireless_node);
            if (xbox_adapter_open(wireless_node, adapter_why,
                                  sizeof adapter_why)) {
                dev->fd = -1;
                dev->wireless_adapter = 1;
                dev->vendor = XBOX_WIRELESS_VID;
                dev->product = XBOX_WIRELESS_PID;
                snprintf(dev->node, sizeof dev->node, "%s", wireless_node);
                snprintf(why, (size_t)whylen, "%s", adapter_why);
                return dev;
            }
#endif
            snprintf(why, (size_t)whylen, "%s: %s",
                     wireless_node, adapter_why);
        } else {
            snprintf(why, (size_t)whylen,
                     "no Xbox-protocol controller connected");
        }
        return NULL;
    }

    /* The descriptor's own length, not the buffer's: walking past the end
     * reads uninitialised stack and can invent endpoints that do not exist. */
    if (!scan_layout(dev, cfg, cfg_len)) {
        snprintf(why, (size_t)whylen, "%s has no usable GIP endpoints", dev->node);
        close(dev->fd);
        dev->fd = -1;
        return NULL;
    }

    if (!bring_up(dev)) {
        snprintf(why, (size_t)whylen, "transfer setup failed: %s", strerror(errno));
        close(dev->fd);
        dev->fd = -1;
        return NULL;
    }

    snprintf(why, (size_t)whylen, "interface %d, IN %02X, OUT %02X",
             dev->iface, dev->ep_in, dev->ep_out);
    return dev;
}

int gip_rebuild(gip_device *dev)
{
    struct usb_fs_uninit un;

    if (dev->wireless_adapter) return 0;
    memset(&un, 0, sizeof un);
    ioctl(dev->fd, USB_FS_UNINIT, &un);
    close(dev->fd);

    dev->fd = open(dev->node, O_RDWR);
    if (dev->fd < 0) return 0;

    return bring_up(dev);
}

void gip_close(gip_device *dev)
{
    struct usb_fs_uninit un;

    if (!dev) return;
    gip_clear_chunk(&dev->chunk);

    if (dev && dev->wireless_adapter) {
        xbox_adapter_close();
        dev->wireless_adapter = 0;
        return;
    }
    if (dev->fd < 0) return;

    memset(&un, 0, sizeof un);
    ioctl(dev->fd, USB_FS_UNINIT, &un);
    close(dev->fd);
    dev->fd = -1;
}

const char *gip_node(const gip_device *dev)    { return dev->node; }
unsigned    gip_vendor(const gip_device *dev)  { return dev->vendor; }
unsigned    gip_product(const gip_device *dev) { return dev->product; }
int gip_is_wireless_adapter(const gip_device *dev)
{
    return dev != NULL && dev->wireless_adapter;
}
