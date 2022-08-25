/*
 *
 * (C) Copyright 2011 Heino Gutschmidt
 * (C) 2011-2015 SYS TEC electronic GmbH, Daniel Krueger, Alexander Stein
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/if_arp.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/printk.h>
#include <asm/unaligned.h>


#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#define SYSTEC_MODULE_VERSION   "0.9.4"
#define SYSTEC_MODULE_DESC      "SYS TEC electronic USB-CANmodul Series Driver " \
                                SYSTEC_MODULE_VERSION

#define SYSTEC_DEBUG_DRIVER         1
#define SYSTEC_DEBUG_CMD_CB         2
#define SYSTEC_DEBUG_STAT_CB        4
#define SYSTEC_DEBUG_DATA_CB        8
#define SYSTEC_DEBUG_TX             16
#define SYSTEC_DEBUG_BITTIMING      32
#define SYSTEC_DEBUG_CMD            64
#define SYSTEC_DEBUG_ERR_HANDLING   128
#define SYSTEC_DEBUG_USB_DISCONNECT 256

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
#define TX_ECHO_SKB_MAX 20
#else
#define TX_ECHO_SKB_MAX 4
#endif
#define MAX_TX_URBS 10
#define MAX_RX_URBS 10
#define RX_BUFFER_SIZE      64
#define INTR_IN_BUFFER_SIZE 4

/* size of command */
#define SYSTEC_CMD_SIZE     8

/* number of endpoints */
#define SYSTEC_NUM_ENDPOINTS 5

#define SYSTEC_EP_DATA_OUT 0
#define SYSTEC_EP_MSG_OUT  1
#define SYSTEC_EP_DATA_IN  2
#define SYSTEC_EP_MSG_IN   3
#define SYSTEC_EP_STAT_IN  4

/* for can socket internal calculation, 48MHz clock */
#define SYSTEC_DEVICE_CLOCK 48000000

/* timeout for bulk transfers - 1000ms */
#define SYSTEC_BULK_TIMEOUT     1000

/* polling time of int (status) urbs - 1ms */
#define SYSTEC_INT_URB_POLL_TIME        1

#define SYSTEC_CAN_CHANNEL              0x00000001
#define SYSTEC_CAN_STATE_MASK           0xFFFF0000
#define SYSTEC_CAN_STATE_ERR_WARN       0x00040000
#define SYSTEC_CAN_STATE_ERR_PASS       0x00080000
#define SYSTEC_CAN_STATE_ERR_BOFF       0x00100000
#define SYSTEC_USB_MASK                 0x0000FFFE
#define SYSTEC_USB_ERR_STATUS_TIMEOUT   0x00002000
#define SYSTEC_USB_ERR_WDT_RESET        0x00004000


#define USBCAN_VRREQ_READ_VERSION       0xB0    // IN:  dwVersion
#define USBCAN_VRREQ_START_UPDATE       0xB1    // OUT: void
#define USBCAN_VRREQ_WRITE_UPDATE       0xB2    // OUT: INTEL-HEX-RECORD
#define USBCAN_VRREQ_STOPP_UPDATE       0xB3    // OUT: void
#define USBCAN_VRREQ_CHECK_UPDATE       0xB4    // IN:  dwErrorCode, dwCryptCrc32, dwReserve
#define USBCAN_VRREQ_WRITE_FLASH        0xB5    // OUT: dwBase, dwSize, fReset
#define USBCAN_VRREQ_RECONNECT          0xB6    // OUT: void
#define USBCAN_VRREQ_READ_FW_VERSION    0xB9    // IN:  dwVersion   // always means FwVersion of Firmware (not supported in Firmware < V4.06)
#define USBCAN_VRREQ_CHECK_CRC          0xBA    // OUT: dwBase, dwSize
#define USBCAN_VRREQ_GET_CRC_RESULT     0xBB    // IN:  dwErrorCode, dwCryptCrc32, dwReserve

/* from Include/Ucanmcpc.h */
#define USBCAN_CMD_INITIALIZE           1
#define USBCAN_CMD_SHUTDOWN             4
#define USBCAN_CMD_RESET                5
#define USBCAN_CMD_READEEPROM           6
#define USBCAN_CMD_WRITEEEPROM          7
#define USBCAN_CMD_SETAMR               11      // -> dwAMR
#define USBCAN_CMD_SETACR               12      // -> dwACR
#define USBCAN_CMD_SETCANMODE           13      // -> bMode
#define USBCAN_CMD_SETBAUDRATE_EX       25
#define USBCAN_CMD_RESET_HW             26
#define USBCAN_CMD_SET_TX_TIMEOUT       30

#define USBCAN_DATAFF_DLC               0x0F

#define USBCAN_DATAFF_POS_CHANNEL       4
#define USBCAN_DATAFF_CHANNEL1          (1 << USBCAN_DATAFF_POS_CHANNEL)

#define USBCAN_MODE_NORMAL              0x00
#define USBCAN_MODE_LISTEN_ONLY         0x01
#define USBCAN_MODE_TX_ECHO             0x02

#define USBCAN_CANID_TX_ECHO_BIT        0x01
#define USBCAN_CANID_TX_ECHO_MASK       0x03

#define USBCAN_CANID_EXT_SHIFT          0x03
#define USBCAN_CANID_STD_SHIFT          0x05

#define USBCAN_DATAFF_ENDOFRESET            0x0F
#define USBCAN_DATACANID_ENDOFRESET         0xF0FF  // motorola format
#define USBCAN_DATACANID_TIMESTAMP2         0xFCFF

/* from Host_GENERIC/Include/Usbcan32.h */
#define USBCAN_MSG_FF_EXT       0x80
#define USBCAN_MSG_FF_RTR       0x40

#define USBCAN_RESET_NO_TXBUFFER_FW         0x00000080          // no TX message buffer reset at firmware level
#define USBCAN_RESET_ONLY_TXBUFFER_FW       (0x0000FFFF & ~(USBCAN_RESET_NO_TXBUFFER_FW))

/* from Host_GENERIC/Library/Ucancmd.c */
#define USBCAN_EEPROM_PID                   0x03
#define USBCAN_EEPROM_SERIALNR              0x08
#define USBCAN_EEPROM_DEVICENR              0x0c
#define USBCAN_EEPROM_ADDR_STATUS_TIMEOUT   0x11 // sizeof(DWORD)    // intel format

/* from Include/Vermco.h */
#define GETPC_MAJOR_VER(ver)            ((ver) & 0x000000FF)
#define GETPC_MINOR_VER(ver)            (((ver) & 0x0000FF00) >> 8)
#define GETPC_RELEASE_VER(ver)          (((ver) & 0xFFFF0000) >> 16)

#define FIRMWARE_FILE(type) \
        "systec_can-" __stringify(type) ".fw"

#define FW_VERSION(major, minor, release)   (major | (minor << 8) | (release << 16))

/* name of firmware image */
#define FIRMWARE_NAME "systec_can-%.4x.fw"
#define BOOTLOADER_NAME "systec_can-bl-%.4x.fw"

#define SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO             0x01
#define SYSTEC_CAN_CHANNEL_FLAGS_CLEAR_TX            0x02    /* This flag is set when the clear buffer mode is enabled */

#define SYSTEC_TYPE_FW 0x00
#define SYSTEC_TYPE_BL 0x01

static struct can_bittiming_const systec_can_bittiming_const = {
        .name = KBUILD_MODNAME,
        .tseg1_min = 1,
        .tseg1_max = 16,
        .tseg2_min = 1,
        .tseg2_max = 8,
        .sjw_max = 4,
        .brp_min = 1,
        .brp_max = 255, /* if > 127 the CLK flag is used */
        .brp_inc = 1,
};

struct firmware_header
{
    u8     header_version;
    u8     type;

    u8     fw_major;
    u8     fw_minor;
    __le16 fw_release_le;
    __le16 product_id_le;

    __le32 fw_address_le;
    __le32 buffer_size_le;
} __packed;

static const struct usb_device_id systec_can_main_table[] = {
        {USB_DEVICE(0x0878, 0x1103)},   /* 8 or 16 channel device generation 3 */
        {USB_DEVICE(0x0878, 0x1104)},   /* single channel device generation 3 */
        {USB_DEVICE(0x0878, 0x1105)},   /* dual channel device generation 3 */
        {USB_DEVICE(0x0878, 0x1121)},   /* dual channel device generation 4 */
        {USB_DEVICE(0x0878, 0x1122)},   /* single channel device generation 4 */
        {USB_DEVICE(0x0878, 0x1145)},   /* 3204013 */
        {USB_DEVICE(0x0878, 0x1101)},   /* running device (normal Windows driver) */
        {USB_DEVICE(0x0878, 0x1181)},   /* running device (Windows network driver) */
        {} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, systec_can_main_table);

/* work structure to schedule commands */
struct systec_work {
        struct work_struct work;
        struct systec_can_chan *chan;
        enum can_state chan_state;
};

struct systec_can_shared;

/* ressources per channel */
struct systec_can_chan {
        struct can_priv can; /* must be the first member */

        /* points to shared ressources between both channels */
        struct systec_can_shared *shared_res;

        /* ressources per channel */
        u8 chan_no;     /* channel number */

        /* restart necessary because of bus-off */
        bool restart_necessary;

        /* internal device status delivered by status endpoint */
        u32 mod_status_int;

        /* current bittiming (extended baudrate register) */
        u32 baud_ex_reg;

        struct {
                /* variables which hold the circular buffer */
                int echo_put;
                int echo_get;
                atomic_t pending;

                /* variables for URB management */
                atomic_t active_tx_urbs;
                struct usb_anchor tx_submitted;
        } tx;

        /* corresponding net device */
        struct net_device *netdev;

        /* prepared work to restart channel (bus-off) within timer function */
        struct systec_work restart_work;

        /* mode flags for e.g. txecho */
        int flags;

        /* timeout in ms when unsendable messages shall be deleted */
        u32 tx_timeout_ms;
};

/* shared ressources between both channels */
struct systec_can_shared {
        /* serial number (device eeprom) */
        u8 serial_no[4];

        /* device number (device eeprom) */
        u8 device_no;

        /* usb device */
        struct usb_device *udev;

        /* reference to private data of each channel */
        struct systec_can_chan *channels[2];

        struct usb_anchor rx_submitted;
        int channels_running;

        struct urb *intr_urb;
        u8 *intr_in_buffer;

        u8 *tx_cmd_buffer;
        u8 *rx_cmd_buffer;
        struct mutex cmd_pending;

        u8 endpoint_addresses[SYSTEC_NUM_ENDPOINTS];
};

/* command message: driver -> CAN-Modul */
struct __attribute__ ((packed)) systec_cmd_msg {
        u32 size;
        u8  cmd_data[SYSTEC_CMD_SIZE];
};

/* CAN message with standard ID */
struct __attribute__ ((packed)) systec_can_std_msg {
    u16  can_id;
    u8   data[8];
    u8   reserved[2];
};

/* CAN message with extended ID */
struct __attribute__ ((packed)) systec_can_ext_msg {
    u32  can_id;
    u8   data[8];
};

/* the internal CAN message */
struct __attribute__ ((packed)) systec_can_msg {
        u8   format;    /* first byte is always the format byte */
        union {         /* handle both types of ID              */
                struct  systec_can_std_msg can_std_msg;
                struct  systec_can_ext_msg can_ext_msg;
        } msg;
        u8   timestamp[3]; /* last 3 bytes are the timestamp */
};

/* binary Intel hex record from usbcan_loader.h */
#define MAX_INTEL_HEX_RECORD_LENGTH 16
struct __attribute__ ((packed)) intel_hex_record
{
    u8 length;
    u16 address;
    u8 type;
    u8  data[MAX_INTEL_HEX_RECORD_LENGTH];

};

/* this hex record type indicates a header record */
#define HEX_RECORD_TYPE_HEADER 0xff


/* prototypes */
static void systec_can_main_disconnect(struct usb_interface *intf);
static int systec_can_main_probe(struct usb_interface *intf,
                         const struct usb_device_id *id);
static int systec_can_main_probe_boot(struct usb_interface *intf,
                         const struct usb_device_id *id);
static int systec_can_open(struct net_device *netdev);
static int systec_can_close(struct net_device *netdev);
static netdev_tx_t systec_can_start_xmit(struct sk_buff *skb, struct net_device *netdev);
static int systec_can_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static int systec_can_set_bittiming(struct net_device *netdev);
static int systec_can_set_mode(struct net_device *netdev, enum can_mode mode);
static void systec_unlink_urbs(struct systec_can_chan *chan);
static int systec_can_send_cmd(struct systec_can_shared *shared_res, struct systec_cmd_msg *msg);

static void systec_dump_cmd_buf(struct systec_can_shared *shared_res,u8 *buffer);
static void systec_dump_msg_buf(struct systec_can_shared *shared_res,u8 *buffer);
static void systec_dump_buf(struct systec_can_shared *shared_res,u8 *buffer, int size);

static void systec_can_read_stat_callback(struct urb *urb);
static void systec_can_read_data_callback(struct urb *urb);
static void systec_can_write_data_callback(struct urb *urb);
static void systec_can_rx_can_msg(struct systec_can_chan *chan, u8 *msg_buf);
static int systec_dual_chan_device(struct systec_can_shared *shared_res);
static int systec_status_timeout(struct systec_can_shared *shared_res, u32* status_timeout);
static void systec_handle_status_change(struct systec_can_chan *chan, u32 mod_status_int_new);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#define get_can_dlc(i)  (min_t(__u8, (i), 8))
static struct sk_buff *alloc_can_skb(struct net_device *dev, struct can_frame **cf);
static struct sk_buff *alloc_can_err_skb(struct net_device *dev, struct can_frame **cf);
#endif
static void systec_work_chan_restart(struct work_struct *work);
static int systec_get_serial_no(struct systec_can_shared *shared_res);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
static inline const char *netdev_name(const struct net_device *dev)
{
       if (dev->reg_state != NETREG_REGISTERED)
               return "(unregistered net_device)";
       return dev->name;
}

#define netdev_printk(level, netdev, format, args...)          \
       dev_printk(level, (netdev)->dev.parent,                 \
                  "%s: " format,                               \
                  netdev_name(netdev), ##args)

#define netdev_err(dev, format, args...)                       \
       netdev_printk(KERN_ERR, dev, format, ##args)
#define netdev_warn(dev, format, args...)                      \
       netdev_printk(KERN_WARNING, dev, format, ##args)
#define netdev_notice(dev, format, args...)                    \
       netdev_printk(KERN_NOTICE, dev, format, ##args)
#define netdev_info(dev, format, args...)                      \
       netdev_printk(KERN_INFO, dev, format, ##args)

#if defined(DEBUG)
#define netdev_dbg(__dev, format, args...)                     \
       netdev_printk(KERN_DEBUG, __dev, format, ##args)
#else
#define netdev_dbg(__dev, format, args...)                     \
({                                                             \
       if (0)                                                  \
               netdev_printk(KERN_DEBUG, __dev, format, ##args); \
       0;                                                      \
})
#endif
#endif


/* the usb handles to operate the CAN device */
static struct usb_driver systec_can_driver = {
        .name = KBUILD_MODNAME,
        .probe = systec_can_main_probe,
        .disconnect = systec_can_main_disconnect,
        .id_table = systec_can_main_table,
        .soft_unbind = 1	/* we want to disable receiver on unbind */
};

/* the network handles to operate the CAN device */
static const struct net_device_ops systec_can_netdev_ops = {
        .ndo_open = systec_can_open,
        .ndo_stop = systec_can_close,
        .ndo_start_xmit = systec_can_start_xmit,
        .ndo_do_ioctl = systec_can_ioctl,
};

/* command work queue */
static struct workqueue_struct *cmd_wq;

static unsigned int debug = 0;
static bool hw_txecho = false;

#define systec_netdev_dbg(type, dev, format, args...) \
do { \
    if (debug & type) \
    { \
        netdev_printk(KERN_DEBUG, dev, format, ##args); \
    } \
} while(0)

#define systec_dev_dbg(type, dev, format, args...) \
do { \
    if (debug & type) \
    { \
        dev_printk(KERN_DEBUG, dev, format, ##args); \
    } \
} while(0)

static int systec_dbg(int type, const char *fmt, ...)
{
        struct va_format vaf;
        va_list args;
        int r = 0;

        if (debug & type)
        {
                va_start(args, fmt);

                vaf.fmt = fmt;
                vaf.va = &args;

                r = printk("%s%pV", KERN_DEBUG, &vaf);
                va_end(args);
        }
        return r;
}

static int systec_setup_tx_timeout(struct systec_can_chan *chan)
{
        struct systec_cmd_msg cmd;
        struct systec_can_shared *shared_res = chan->shared_res;

        int ret_val;

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));

        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SET_TX_TIMEOUT;
        put_unaligned_le32(chan->tx_timeout_ms, &cmd.cmd_data[1]);
        cmd.cmd_data[6] = chan->chan_no;

        ret_val = systec_can_send_cmd(shared_res, &cmd);

        return ret_val;
}

static int systec_shared_start_urbs(struct systec_can_shared *shared_res, struct net_device *netdev)
{
        int err;
        int i;
        struct urb *new_urb;
        u8 *new_buf;

        if (shared_res->channels_running++ > 0)
                return (0);

        /* create the rx URBs since there is not any URB running */
        for (i = 0; i < MAX_RX_URBS; i++) {
                new_urb = NULL;
                new_buf = NULL;

                /* alloc the URB */
                new_urb = usb_alloc_urb(0, GFP_KERNEL);
                if (!new_urb) {
                        netdev_err(netdev,
                                "not enough memory for URB\n");

                        return (-ENOMEM);
                }

                /* create the buffer for the URB */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
                new_buf = usb_alloc_coherent(shared_res->udev, RX_BUFFER_SIZE, GFP_KERNEL,
                                           &new_urb->transfer_dma);
#else
                new_buf = usb_buffer_alloc(shared_res->udev, RX_BUFFER_SIZE, GFP_KERNEL,
                                           &new_urb->transfer_dma);
#endif

                if (!new_buf) {
                        netdev_err(netdev,
                                "not enough memory for URB buffer\n");

                        usb_free_urb(new_urb);
                        return (-ENOMEM);
                }

                usb_fill_bulk_urb(new_urb, shared_res->udev,
                                 usb_rcvbulkpipe(shared_res->udev,
                                    shared_res->endpoint_addresses[SYSTEC_EP_DATA_IN]),
                                 new_buf, RX_BUFFER_SIZE,
                                 systec_can_read_data_callback, shared_res);
                new_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
                usb_anchor_urb(new_urb, &shared_res->rx_submitted);

                err = usb_submit_urb(new_urb, GFP_KERNEL);
                if (err) {

                        usb_unanchor_urb(new_urb);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
                        usb_free_coherent(shared_res->udev, RX_BUFFER_SIZE, new_buf,
                                        new_urb->transfer_dma);
#else
                        usb_buffer_free(shared_res->udev, RX_BUFFER_SIZE, new_buf,
                                        new_urb->transfer_dma);
#endif
                        usb_free_urb(new_urb);
                        break;
                }

                /* the driver is finished with the URB */
                usb_free_urb(new_urb);
        }


        /* check if one urb could be submited at least */
        if (i == 0) {
                netdev_err(netdev, "could not submit any read data urb\n");

                return (err);
        }

        if (i < MAX_RX_URBS) {
                netdev_warn(netdev, "only %d of %d urbs could be submitted (err = %d)\n", i, MAX_RX_URBS, err);
        }

        /* setup status urb */
        new_urb = usb_alloc_urb(0, GFP_KERNEL);
        if (new_urb) {
                shared_res->intr_urb = new_urb;

                usb_fill_int_urb(new_urb, shared_res->udev,
                                 usb_rcvintpipe(shared_res->udev,
                                shared_res->endpoint_addresses[SYSTEC_EP_STAT_IN]),
                                shared_res->intr_in_buffer,
                                INTR_IN_BUFFER_SIZE,
                                systec_can_read_stat_callback, shared_res,
                                SYSTEC_INT_URB_POLL_TIME);

                err = usb_submit_urb(shared_res->intr_urb, GFP_KERNEL);
                if (err) {
                        usb_free_urb(new_urb);
                        shared_res->intr_urb = NULL;

                        netdev_err(netdev, "intr URB submit failed: %d\n",
                                err);

                        return (err);
                }
        }
        else {
                netdev_err(netdev, "Couldn't alloc intr URB\n");

                return (-ENOMEM);
        }

        return (0);
}

static void systec_shared_stop_urbs(struct systec_can_shared *shared_res, struct net_device *netdev)
{
        if (--shared_res->channels_running > 0)
                return;

        /* stop status urb */
        if (shared_res->intr_urb) {
                usb_kill_urb(shared_res->intr_urb);
                usb_free_urb(shared_res->intr_urb);
                shared_res->intr_urb = NULL;
                systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "%s: intr_urb stopped\n", __func__);
        }

        /* stop rx urb */
        usb_kill_anchored_urbs(&shared_res->rx_submitted);
        systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "%s: rx_urb stopped\n", __func__);
}

static int systec_can_open(struct net_device *netdev)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        int err;
        struct systec_cmd_msg cmd;
        int ret_val;

        systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "systec_can_open\n");

        /* open CAN device */
        err = open_candev(netdev);
        if (err) {
                return err;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
        can_led_event(netdev, CAN_LED_EVENT_OPEN);
#endif

        chan->mod_status_int &= ~SYSTEC_CAN_STATE_MASK;
        err = systec_shared_start_urbs(chan->shared_res, netdev);
        if (err) {
                goto failed;
        }

        /* clear can status */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_RESET;
        cmd.cmd_data[6] = chan->chan_no;
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* set AMR */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SETAMR;
        cmd.cmd_data[6] = chan->chan_no;
        put_unaligned_be32(0xffffffff, &cmd.cmd_data[1]);
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* set ACR */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SETACR;
        cmd.cmd_data[6] = chan->chan_no;
        put_unaligned_be32(0x0, &cmd.cmd_data[1]);
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* set CAN mode */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SETCANMODE;
        cmd.cmd_data[1] = USBCAN_MODE_NORMAL; /* standard r/w mode */
        if (chan->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
                cmd.cmd_data[1] |= USBCAN_MODE_LISTEN_ONLY;
        if (hw_txecho) {
                cmd.cmd_data[1] |= USBCAN_MODE_TX_ECHO;
                chan->flags |= SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO;
        } else {
                chan->flags &= ~SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO;
        }
        cmd.cmd_data[6] = chan->chan_no;
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* start device */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_INITIALIZE;
        cmd.cmd_data[6] = chan->chan_no;
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* setup Tx timeout which is cleared after USBCAN_CMD_INITIALIZE */
        if (systec_dual_chan_device(chan->shared_res))
        {
                ret_val = systec_setup_tx_timeout(chan);
        }

        chan->can.state = CAN_STATE_ERROR_ACTIVE;

        netif_start_queue(netdev);

        return (0);

failed:
        systec_shared_stop_urbs(chan->shared_res, netdev);

        return err;
}

/*
  handler of status endpoint (interrupt endpoint)
*/
static void systec_can_read_stat_callback(struct urb *urb)
{
        struct systec_can_shared *shared_res = urb->context;
        struct systec_can_chan *chan;
        u32 mod_status;
        int ret_stat;

        if (urb->status == 0) {
                mod_status = be32_to_cpu(*((u32*)urb->transfer_buffer));

                chan = shared_res->channels[(mod_status & SYSTEC_CAN_CHANNEL)];

                /* clear channel number bit */
                mod_status &= ~SYSTEC_CAN_CHANNEL;

                if ((mod_status != chan->mod_status_int)
                    && netif_device_present(chan->netdev)
                    && netif_running(chan->netdev)) {
                        /* channel status has been changed */
                        systec_dev_dbg(SYSTEC_DEBUG_STAT_CB, &shared_res->udev->dev, "systec_can_read_stat_callback (%lu): %d, %x\n", jiffies,
                                urb->status, mod_status);

                        systec_dump_buf(shared_res, urb->transfer_buffer, urb->actual_length);

                        /* handle status */
                        systec_handle_status_change(chan, mod_status);

                        /* update internal CAN status */
                        chan->mod_status_int = mod_status;
                }

                /* resubmit status urb */
                ret_stat = usb_submit_urb(urb, GFP_ATOMIC);
        }


/*
  todo: error to which device ?
        if (ret_stat == -ENODEV) {
                netif_device_detach(netdev);
        }
        else {
                if (ret_stat) {
                        netdev_err(netdev,
                        "failed resubmitting intr urb: %d\n", ret_stat);
                }
        }
*/

        return;

}


static void systec_handle_status_change(struct systec_can_chan *chan, u32 mod_status_int_new)
{
        struct can_frame *cf;
        struct sk_buff *skb;
        struct net_device_stats *stats = &chan->netdev->stats;
        enum can_state new_state = chan->can.state;


        systec_netdev_dbg(SYSTEC_DEBUG_ERR_HANDLING, chan->netdev, "systec_handle_status_change (%lu): %x\n", jiffies, chan->mod_status_int);

        /* convert device status into CAN state */
        if (mod_status_int_new & SYSTEC_CAN_STATE_ERR_BOFF) {
                new_state = CAN_STATE_BUS_OFF;
                chan->restart_necessary = 1;
        }
        else if (!chan->restart_necessary) {
                if (mod_status_int_new & SYSTEC_CAN_STATE_ERR_PASS) {
                        new_state = CAN_STATE_ERROR_PASSIVE;
                }
                else {
                        if (mod_status_int_new & SYSTEC_CAN_STATE_ERR_WARN) {
                                new_state = CAN_STATE_ERROR_WARNING;
                        }
                        else {
                                new_state = CAN_STATE_ERROR_ACTIVE;
/*
  todo: check other states
*/
                        }
                }
        }

        systec_netdev_dbg(SYSTEC_DEBUG_ERR_HANDLING, chan->netdev, "   new_state: %x, current: %x\n", new_state, chan->can.state);

        if ((mod_status_int_new ^ chan->mod_status_int)
            & (SYSTEC_USB_ERR_STATUS_TIMEOUT | SYSTEC_USB_ERR_WDT_RESET))
        {   // Generic channel independent state changed
            if ((mod_status_int_new & SYSTEC_USB_ERR_STATUS_TIMEOUT)
                && ((chan->mod_status_int & SYSTEC_USB_ERR_STATUS_TIMEOUT) == 0))
            {
                netdev_warn(chan->netdev, "device has been reset because of Status Timeout.\n");
            }
            if ((mod_status_int_new & SYSTEC_USB_ERR_WDT_RESET)
                && ((chan->mod_status_int & SYSTEC_USB_ERR_WDT_RESET) == 0))
            {
                netdev_warn(chan->netdev, "device has been reset by Watchdog.\n");
            }
            if (new_state == chan->can.state)
            {
                    return;
            }
        }
        else if (new_state == chan->can.state)
        {
                systec_netdev_dbg(SYSTEC_DEBUG_ERR_HANDLING, chan->netdev, "systec_handle_status_change: unknown status change 0x%X\n", mod_status_int_new);
                return;
        }

        /* alloc error frame */
        skb = alloc_can_err_skb(chan->netdev, &cf);
        if (unlikely(skb == NULL)) {
                return;
        }

        switch(chan->can.state) {
        case CAN_STATE_ERROR_ACTIVE:
                /* to ERROR_WARNING, ERROR_PASSIVE, BUS_OFF */
                if (new_state >= CAN_STATE_ERROR_WARNING && new_state <= CAN_STATE_BUS_OFF) {
                        chan->can.can_stats.error_warning++;

                        cf->can_id |= CAN_ERR_CRTL;
                        cf->data[1] = CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_RX_WARNING;
                }

        case CAN_STATE_ERROR_WARNING:
                /* to ERROR_PASSIVE, BUS_OFF */
                if (new_state >= CAN_STATE_ERROR_PASSIVE && new_state <= CAN_STATE_BUS_OFF) {
                        chan->can.can_stats.error_passive++;

                        cf->can_id |= CAN_ERR_CRTL;
                        cf->data[1] = CAN_ERR_CRTL_TX_PASSIVE | CAN_ERR_CRTL_RX_PASSIVE;
                        chan->can.can_stats.error_passive++;
                }
                break;

         case CAN_STATE_BUS_OFF:
                 /* to ERROR_ACTIVE, ERROR_WARNING, ERROR_PASSIVE */
                 if (new_state <= CAN_STATE_ERROR_PASSIVE) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8, 0)
                        schedule_delayed_work(&chan->can.restart_work, jiffies);
#else
                         mod_timer(&chan->can.restart_timer, jiffies);
#endif
                         netif_wake_queue(chan->netdev);
                 }
                 break;

        default:
                break;
        }

        switch (new_state) {
        case CAN_STATE_ERROR_ACTIVE:
                cf->can_id |= CAN_ERR_PROT;
                cf->data[2] = CAN_ERR_PROT_ACTIVE;

                break;
        case CAN_STATE_BUS_OFF:
                cf->can_id |= CAN_ERR_BUSOFF;
                netif_carrier_off(chan->netdev);
                chan->can.can_stats.bus_off++;
                 if (chan->can.restart_ms) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8, 0)
                        schedule_delayed_work(&chan->can.restart_work,
                                              msecs_to_jiffies(chan->can.restart_ms));
#else
                         /* schedule channel restart (will call can.do_set_mode) */
                         mod_timer(&chan->can.restart_timer,
                                 jiffies + msecs_to_jiffies(chan->can.restart_ms));
#endif
                 }
         default:
                break;
        }

        chan->can.state = new_state;

        stats->rx_packets++;
        stats->rx_bytes += cf->can_dlc;
        netif_rx(skb);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
        can_led_event(chan->netdev, CAN_LED_EVENT_RX);
#endif
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)

static struct sk_buff *alloc_can_skb(struct net_device *dev, struct can_frame **cf)
{
        struct sk_buff *skb;

        skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
        if (unlikely(!skb))
                return NULL;

        skb->protocol = htons(ETH_P_CAN);
        skb->pkt_type = PACKET_BROADCAST;
        skb->ip_summed = CHECKSUM_UNNECESSARY;
        *cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));
        memset(*cf, 0, sizeof(struct can_frame));

        return skb;
}

static struct sk_buff *alloc_can_err_skb(struct net_device *dev, struct can_frame **cf)
{
        struct sk_buff *skb;

        skb = alloc_can_skb(dev, cf);
        if (unlikely(!skb))
                return NULL;

        (*cf)->can_id = CAN_ERR_FLAG;
        (*cf)->can_dlc = CAN_ERR_DLC;

        return skb;
}

#endif

static void systec_can_tx_echo(struct systec_can_chan *chan, u8 dlc)
{
        if (atomic_read(&chan->tx.pending) <= 0) {
                WARN_ON(1);
                return;
        }
        chan->netdev->stats.tx_packets++;
        chan->netdev->stats.tx_bytes += dlc;

        can_get_echo_skb(chan->netdev, chan->tx.echo_get);

        chan->tx.echo_get++;
        if (chan->tx.echo_get >= TX_ECHO_SKB_MAX)
                chan->tx.echo_get = 0;
        atomic_dec(&chan->tx.pending);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
        can_led_event(chan->netdev, CAN_LED_EVENT_TX);
#endif
}

static void systec_can_rx_can_msg(struct systec_can_chan *chan, u8 *msg_buf)
{
        struct can_frame *cf;
        struct sk_buff *skb;
        struct systec_can_msg *msg;
        int i;
        struct net_device_stats *stats = &chan->netdev->stats;
        u8 dlc;

        msg = (struct systec_can_msg *)msg_buf;

        /* If we do hw tx_echo... */
        if (chan->flags & SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO) {
                u32 id;

                /* ... check if we have an echoed message... */
                if (msg->format & USBCAN_MSG_FF_EXT) {
                        id = get_unaligned_be32(&msg->msg.can_ext_msg.can_id);
                } else {
                        id = (u32) get_unaligned_be16(&msg->msg.can_std_msg.can_id);
                }
                /* ... and send the stored SKB back */
                if (id & USBCAN_CANID_TX_ECHO_MASK) {
                        dlc = get_can_dlc(msg->format & USBCAN_DATAFF_DLC);
                        systec_can_tx_echo(chan, dlc);

                        if ((atomic_read(&chan->tx.active_tx_urbs) < MAX_TX_URBS) && netif_queue_stopped(chan->netdev))
                                netif_wake_queue(chan->netdev);

                        /* echo done, just return */
                        return;
                }
        }

        skb = alloc_can_skb(chan->netdev, &cf);
        if (skb == NULL) {
                return;
        }

        /* get size of data part of CAN message */
        cf->can_dlc = get_can_dlc(msg->format & USBCAN_DATAFF_DLC);

        cf->can_id = 0;

        if (msg->format & USBCAN_MSG_FF_RTR) {
                /* it is a RTR frame */
                cf->can_id |= CAN_RTR_FLAG;
        }
        else {
                if (msg->format & USBCAN_MSG_FF_EXT) {
                        /* copy data of an extended frame */
                        for (i = 0; i < cf->can_dlc; i++) {
                                cf->data[i] = msg->msg.can_ext_msg.data[i];
                        }

                }
                else {
                        /* copy data of a standard frame */
                        for (i = 0; i < cf->can_dlc; i++) {
                                cf->data[i] = msg->msg.can_std_msg.data[i];
                        }
                }
        }

        /* copy CAN ID */
        if (msg->format & USBCAN_MSG_FF_EXT) {
                /* extended frame */
                cf->can_id |= CAN_EFF_FLAG;

                cf->can_id |= get_unaligned_be32(&msg->msg.can_ext_msg.can_id) >> USBCAN_CANID_EXT_SHIFT;
        }
        else {
                /* standard frame */
                cf->can_id |= (u32) (get_unaligned_be16(&msg->msg.can_std_msg.can_id) >> USBCAN_CANID_STD_SHIFT);
        }

        /* deliver frame to socket layer */
        netif_rx(skb);
        stats->rx_packets++;
        stats->rx_bytes += cf->can_dlc;
}

static void systec_can_read_data_callback(struct urb *urb)
{

        struct systec_can_shared *shared_res = urb->context;
        struct systec_can_chan *chan;
        struct net_device *netdev;
        int ret_stat;
        int total_size;
        u8 *msg_buf;
        struct systec_can_msg *msg;


        systec_dev_dbg(SYSTEC_DEBUG_DATA_CB, &shared_res->udev->dev, "systec_can_read_data_callback (%lu): %d\n", jiffies, urb->status);

        if (urb->status) {
                dev_info(&urb->dev->dev, "rx urb aborted (%d)\n",
                         urb->status);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
                usb_free_coherent(urb->dev, urb->transfer_buffer_length,
                                urb->transfer_buffer, urb->transfer_dma);
#else
                usb_buffer_free(urb->dev, urb->transfer_buffer_length,
                                urb->transfer_buffer, urb->transfer_dma);
#endif

                return;
        }

        if (urb->status == 0) {
                systec_dev_dbg(SYSTEC_DEBUG_DATA_CB, &shared_res->udev->dev, "    message received: %d bytes\n", urb->actual_length);

                msg_buf = urb->transfer_buffer;
                total_size = urb->actual_length;
                while (total_size >= sizeof(struct systec_can_msg)) {

                        msg = (struct systec_can_msg *)msg_buf;
                        systec_dump_msg_buf(shared_res, msg_buf);

                        if ((msg->msg.can_std_msg.can_id == USBCAN_DATACANID_TIMESTAMP2) &&
                             (msg->format & USBCAN_DATAFF_DLC))
                        {
                                // Currently ignore timestamp messages
                        }
                        else
                        {
                                if (msg->format & USBCAN_DATAFF_CHANNEL1) {
                                        if (shared_res->channels[1]) {
                                                chan = shared_res->channels[1];
                                        }
                                        else {
                                                systec_dev_dbg(SYSTEC_DEBUG_DATA_CB, &shared_res->udev->dev, "received data for unavailable channel #2\n");
                                                /* take channel 0 */
                                                chan = shared_res->channels[0];
                                        }
                                }
                                else {
                                        chan = shared_res->channels[0];
                                }
                                netdev = chan->netdev;

                                if (netif_device_present(netdev)
                                && netif_running(netdev)) {
                                        systec_can_rx_can_msg(chan, msg_buf);
                                }
                        }

                        msg_buf += sizeof(struct systec_can_msg);
                        total_size -= sizeof(struct systec_can_msg);
                }
        }

        /* resubmit urb */
        usb_fill_bulk_urb(urb, shared_res->udev,
                          usb_rcvbulkpipe(shared_res->udev, shared_res->endpoint_addresses[SYSTEC_EP_DATA_IN]),
                          urb->transfer_buffer, RX_BUFFER_SIZE,
                          systec_can_read_data_callback, shared_res);

        /* urbs are unanchored upon completion, so reanchor tis urb before submitting */
        usb_anchor_urb(urb, &shared_res->rx_submitted);
        ret_stat = usb_submit_urb(urb, GFP_ATOMIC);
        if (ret_stat) {
                usb_unanchor_urb(urb);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
                usb_free_coherent(urb->dev, urb->transfer_buffer_length,
                                urb->transfer_buffer, urb->transfer_dma);
#else
                usb_buffer_free(urb->dev, urb->transfer_buffer_length,
                                urb->transfer_buffer, urb->transfer_dma);
#endif
        }

/*
  todo: which device should fail?

        if (ret_stat == -ENODEV) {
                netif_device_detach(netdev);
        }
        else {
                if (ret_stat) {
                        netdev_err(netdev,
                                "failed resubmitting read bulk urb: %d\n", ret_stat);
                }
        }
*/

        return;
}

static int systec_can_close(struct net_device *netdev)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        struct systec_cmd_msg cmd;
        int ret_val;

        systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "systec_can_close: channel %d\n", chan->chan_no);

        /* stop channel */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SHUTDOWN;
        cmd.cmd_data[6] = chan->chan_no;
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        /* stop tx */
        netif_stop_queue(netdev);
        systec_unlink_urbs(chan);

        systec_shared_stop_urbs(chan->shared_res, netdev);

        close_candev(netdev);
        chan->can.state = CAN_STATE_STOPPED;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
        can_led_event(netdev, CAN_LED_EVENT_STOP);
#endif

        return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
static inline int can_dropped_invalid_skb(struct net_device *dev,
                                          struct sk_buff *skb)
{
        const struct can_frame *cf = (struct can_frame *)skb->data;

        if (unlikely(skb->len != sizeof(*cf) || cf->can_dlc > 8)) {
                kfree_skb(skb);
                dev->stats.tx_dropped++;
                return 1;
        }

        return 0;
}
#endif

static netdev_tx_t systec_can_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        struct net_device_stats *stats = &netdev->stats;
        struct can_frame *cf = (struct can_frame *)skb->data;
        struct systec_can_msg *msg;
        struct urb *tx_urb;
        u8 *buf;
        int i;
        int ret_val;
        int echo_flag = 0;

        if (likely((chan->flags & SYSTEC_CAN_CHANNEL_FLAGS_CLEAR_TX) == 0)) {
                if (can_dropped_invalid_skb(netdev, skb)) {
                        return NETDEV_TX_OK;
                }
        } else {
                /* Just skip incoming SKBs unless it is the special CAN-ID */
                if ((cf->can_id != USBCAN_DATACANID_ENDOFRESET) ||
                                (cf->can_dlc != USBCAN_DATAFF_ENDOFRESET)) {
                        kfree_skb(skb);
                        return NETDEV_TX_OK;
                }
        }

        systec_netdev_dbg(SYSTEC_DEBUG_TX, netdev, "systec_can_start_xmit (%lu)\n", jiffies);

        /* create the tx urb */
        tx_urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!tx_urb) {
                netdev_err(netdev, "not enough memory to create tx_urb\n");

                dev_kfree_skb(skb);
                stats->tx_dropped++;
                return(NETDEV_TX_OK);
        }

        /* create the buffer for the CAN message */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        buf = usb_alloc_coherent(chan->shared_res->udev, sizeof(struct systec_can_msg),
                               GFP_ATOMIC, &tx_urb->transfer_dma);
#else
        buf = usb_buffer_alloc(chan->shared_res->udev, sizeof(struct systec_can_msg),
                               GFP_ATOMIC, &tx_urb->transfer_dma);
#endif
        if (!buf) {
                netdev_err(netdev, "not enough memory to create tx buffer\n");

                usb_free_urb(tx_urb);
                dev_kfree_skb(skb);
                stats->tx_dropped++;
                return(NETDEV_TX_OK);
        }

        /* map internal CAN message to buffer */
        msg = (struct systec_can_msg *)buf;

        /* set DLC */
        msg->format = cf->can_dlc;

        /* set channel number */
        if (chan->chan_no == 1) {
                /* set bit for channel 1 */
                msg->format |= USBCAN_DATAFF_CHANNEL1;
        }

        /* If we do hw txecho set Bit 0 in CAN-ID Bytes */
        if (chan->flags & SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO)
                echo_flag = USBCAN_CANID_TX_ECHO_BIT;

        /* the CAN ID itself is shifted depending on frame format */
        if (cf->can_id & CAN_EFF_FLAG) {
                put_unaligned_be32(((cf->can_id & CAN_ERR_MASK) << USBCAN_CANID_EXT_SHIFT) | echo_flag,
                        &msg->msg.can_ext_msg.can_id);
                msg->format |= USBCAN_MSG_FF_EXT;
        }
        else {
                if (likely((chan->flags & SYSTEC_CAN_CHANNEL_FLAGS_CLEAR_TX) == 0)) {
                        put_unaligned_be16(((cf->can_id & CAN_ERR_MASK) << USBCAN_CANID_STD_SHIFT) | echo_flag,
                                &msg->msg.can_std_msg.can_id);
                } else {
                        put_unaligned_be16(be16_to_cpu(USBCAN_DATACANID_ENDOFRESET),
                                &msg->msg.can_std_msg.can_id);
                }
        }

        if (cf->can_id & CAN_RTR_FLAG) {
                msg->format |= USBCAN_MSG_FF_RTR;
        }
        else {
                if (cf->can_id & CAN_EFF_FLAG) {
                        for (i = 0; i < cf->can_dlc; i++) {
                                msg->msg.can_ext_msg.data[i] = cf->data[i];
                        }
                }
                else {
                        for (i = 0; i < cf->can_dlc; i++) {
                                msg->msg.can_std_msg.data[i] = cf->data[i];
                        }
                }
        }

        systec_dump_msg_buf(chan->shared_res, buf);

        usb_fill_bulk_urb(tx_urb, chan->shared_res->udev,
                          usb_sndbulkpipe(chan->shared_res->udev,
                                          chan->shared_res->endpoint_addresses[SYSTEC_EP_DATA_OUT]),
                          buf,
                          sizeof(struct systec_can_msg),
                          systec_can_write_data_callback, chan);

        tx_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
        usb_anchor_urb(tx_urb, &chan->tx.tx_submitted);

        can_put_echo_skb(skb, netdev, chan->tx.echo_put);

        atomic_inc(&chan->tx.active_tx_urbs);
        atomic_inc(&chan->tx.pending);
        chan->tx.echo_put++;

        ret_val = usb_submit_urb(tx_urb, GFP_ATOMIC);

        /* prefer else part */
        if(unlikely(ret_val)) {
                atomic_dec(&chan->tx.active_tx_urbs);
                atomic_dec(&chan->tx.pending);
                chan->tx.echo_put--;

                can_free_echo_skb(netdev, chan->tx.echo_put);

                usb_unanchor_urb(tx_urb);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
                usb_free_coherent(chan->shared_res->udev, sizeof(struct systec_can_msg),
                                buf, tx_urb->transfer_dma);
#else
                usb_buffer_free(chan->shared_res->udev, sizeof(struct systec_can_msg),
                                buf, tx_urb->transfer_dma);
#endif
                if (ret_val == -ENODEV) {
                        netif_device_detach(netdev);
                } else {
                        netdev_warn(netdev, "failed tx_urb %d\n", ret_val);

                        stats->tx_dropped++;
                }
        }
        else {
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 6, 0)
                netif_start_queue(netdev);
#else
                netdev->trans_start = jiffies;
#endif
                if ((atomic_read(&chan->tx.active_tx_urbs) >= MAX_TX_URBS)
                                || (atomic_read(&chan->tx.pending) >= TX_ECHO_SKB_MAX)) {
                        /* slow down transmitting frames */
                        netif_stop_queue(netdev);
                }
        }
        if (chan->tx.echo_put >= TX_ECHO_SKB_MAX)
            chan->tx.echo_put = 0;

        usb_free_urb(tx_urb);

        return NETDEV_TX_OK;
}

static void systec_can_write_data_callback(struct urb *urb)
{
        struct systec_can_chan *chan = urb->context;
        struct net_device *netdev;
        u8 dlc;
        struct systec_can_msg *msg;

        WARN_ON(!chan);

        netdev = chan->netdev;

        systec_netdev_dbg(SYSTEC_DEBUG_TX, netdev, "systec_can_write_data_callback (%lu)\n", jiffies);

        /* We assume just one CAN message sent here */
        msg = (struct systec_can_msg *)urb->transfer_buffer;
        dlc = get_can_dlc(msg->format & USBCAN_DATAFF_DLC);

        /* free the buffer of CAN message */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        usb_free_coherent(urb->dev, urb->transfer_buffer_length,
                        urb->transfer_buffer, urb->transfer_dma);
#else
        usb_buffer_free(urb->dev, urb->transfer_buffer_length,
                        urb->transfer_buffer, urb->transfer_dma);
#endif

        /* the urb is not active any more */
        atomic_dec(&chan->tx.active_tx_urbs);

        if (!netif_device_present(netdev)
            || !netif_running(netdev))
                return;

        if (urb->status)
                netdev_info(netdev, "tx urb aborted (%d)\n",
                         urb->status);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 6, 0)
        netif_start_queue(netdev);
#else
        netdev->trans_start = jiffies;
#endif

        /* If we do no hw tx_echo, send SKB back here */
        if (! (chan->flags & SYSTEC_CAN_CHANNEL_FLAGS_TX_ECHO)) {
                systec_can_tx_echo(chan, dlc);
        }

        /* check if the queue has been stopped and we have at least one echo_skb available */
        if (!urb->status && (atomic_read(&chan->tx.pending) < TX_ECHO_SKB_MAX) && netif_queue_stopped(netdev))
                netif_wake_queue(netdev);

}

static int systec_can_set_bittiming(struct net_device *netdev)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        struct can_bittiming *bt = &chan->can.bittiming;
        struct systec_cmd_msg cmd;
        int ret_val;
        u32 baud_ex_reg;

        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "systec_can_set_bittiming (#%d)\n",
               (int)chan->chan_no);

        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  bitrate: %d\n", bt->bitrate);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  sample_point: %x\n", bt->sample_point);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  tq: %x\n", bt->tq);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  prop_seg: %x\n", bt->prop_seg);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  phase_seg1: %x\n", bt->phase_seg1);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  phase_seg2: %x\n", bt->phase_seg2);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  sjw: %x\n", bt->sjw);
        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  brp: %x\n", bt->brp);

        /* set PHASE2 */
        baud_ex_reg = (bt->phase_seg2 - 1) & 0x7;

        /* set PHASE1 */
        baud_ex_reg |= ((bt->phase_seg1 - 1) & 0x7) << 4;

        /* set PROPAG */
        baud_ex_reg |= ((bt->prop_seg - 1) & 0x7) << 8;

        /* set BPR */
/*
  todo: what happens if bt->brp is 0x80
*/
        if (bt->brp & 0x80) {
                /* use additional clock devider */
                baud_ex_reg |= (((bt->brp >> 1) - 1) & 0x7f) << 16;

                baud_ex_reg |= 0x80000000;
        }
        else
        {
                /* do no use additional clock devider */
                baud_ex_reg |= ((bt->brp - 1) & 0x7f) << 16;
        }

        if (chan->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) {
                /* set SAM */
                baud_ex_reg |= 0x01000000;
        }

        systec_netdev_dbg(SYSTEC_DEBUG_BITTIMING, netdev, "  baud_ex_reg: %x\n", baud_ex_reg);

        ret_val = 0;
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));

        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SETBAUDRATE_EX;
        cmd.cmd_data[6] = chan->chan_no;

        put_unaligned_le32(baud_ex_reg, &cmd.cmd_data[1]);

        /* save bittiming */
        chan->baud_ex_reg = baud_ex_reg;

        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);
/*
todo: check ret_val, move commands to proper function
*/


        return(ret_val);
}

/**
 * systec_can_clear_tx_buf - Send command to clear channel tx buffer
 * and send "stop" CAN message
 * @netdev:   Network interface device structure
 * Returns
 * 0:              Successfully
 * Negative value: Failed
 */
static int systec_can_clear_tx_buf(struct net_device *netdev)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        struct systec_can_shared *shared_res = chan->shared_res;
        struct systec_cmd_msg cmd;
        int ret_val;
        struct sk_buff *skb;
        struct can_frame *cf;

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));

        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_RESET;
        put_unaligned_be32(USBCAN_RESET_ONLY_TXBUFFER_FW, &cmd.cmd_data[1]);
        cmd.cmd_data[6] = chan->chan_no;

        skb = alloc_can_skb(netdev, &cf);

        if (skb) {
                ret_val = systec_can_send_cmd(shared_res, &cmd);

                if (!ret_val) {
                        /* see can_send (af_can.c) for can specific settings */
                        skb->protocol = htons(ETH_P_CAN);
                        skb_reset_network_header(skb);
                        skb_reset_transport_header(skb);

                        /* struct can_frame already cleared */
                        cf->can_id = USBCAN_DATACANID_ENDOFRESET;
                        cf->can_dlc = USBCAN_DATAFF_ENDOFRESET;

                        ret_val = systec_can_start_xmit(skb, netdev);

                        if (ret_val > 0)
                                ret_val = net_xmit_errno(ret_val);
                }
        } else
                ret_val = -ENOMEM;

        return ret_val;
}

/**
 * systec_can_ioctl - Handle private IOCTLs
 * @netdev:   Network interface device structure
 * @ifr:      Pointer to ifr structure
 * @cmd:      Control command
 * Returns
 *        0:        Successfully
 *        Negative value:        Failed
 */
static int systec_can_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
        struct systec_can_chan *chan = netdev_priv(netdev);
        int err = -EOPNOTSUPP;

        systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "cmd : 0x%04x\n", cmd);

        switch (cmd) {
        case SIOCDEVPRIVATE:
                /* Stop queue, so we can use the URBs */
                netif_stop_queue(netdev);
                /* mark device dropping any tx messages */
                chan->flags |= SYSTEC_CAN_CHANNEL_FLAGS_CLEAR_TX;
                /* Kill outstanding URB transfers*/
                systec_unlink_urbs(chan);
                /* clear tx buffer on hardware */
                err = systec_can_clear_tx_buf(netdev);
                /* read more SKBs from net queue to be discarded */
                netif_wake_queue(netdev);
                break;
        case SIOCDEVPRIVATE + 1:
                chan->flags &= ~SYSTEC_CAN_CHANNEL_FLAGS_CLEAR_TX;
                err = 0;
                break;
        }
        return err;
}

static int systec_get_serial_no(struct systec_can_shared *shared_res)
{
        struct systec_cmd_msg cmd;
        int ret_val;
        int i;

        /* get serial number */
        for (i = 0; i < 4; i++) {
                memset(&cmd, 0, sizeof(struct systec_cmd_msg));
                cmd.size = SYSTEC_CMD_SIZE;
                cmd.cmd_data[0] = USBCAN_CMD_READEEPROM;
                cmd.cmd_data[1] = USBCAN_EEPROM_SERIALNR + i;
                ret_val = systec_can_send_cmd(shared_res, &cmd);

                shared_res->serial_no[i] = cmd.cmd_data[2];
        }


        /* get device number */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_READEEPROM;
        cmd.cmd_data[1] = USBCAN_EEPROM_DEVICENR;
        ret_val = systec_can_send_cmd(shared_res, &cmd);

        shared_res->device_no = cmd.cmd_data[2];

        return(0);
}

static int systec_dual_chan_device(struct systec_can_shared *shared_res)
{
        struct systec_cmd_msg cmd;
        int ret_val;
        u16 product_id;

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_READEEPROM;
        cmd.cmd_data[1] = USBCAN_EEPROM_PID;
        ret_val = systec_can_send_cmd(shared_res, &cmd);

/*
todo: check ret_val
*/

        product_id = (u16)cmd.cmd_data[2];

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_READEEPROM;
        cmd.cmd_data[1] = USBCAN_EEPROM_PID + 1;
        ret_val = systec_can_send_cmd(shared_res, &cmd);

        product_id |= ((u16)cmd.cmd_data[2] << 8);

        if (product_id != 0) {
            /* Bit 0 of product ID is set, if it's a dual channel device */
            return (product_id & 0x0001);
        }

//        netdev_err(chan->netdev, "product id %x unknown (code %d)\n", product_id, ret_val);
        /* assume single channel device */
        return(0);
}

static int systec_status_timeout(struct systec_can_shared *shared_res, u32* status_timeout)
{
        struct systec_cmd_msg cmd;
        int ret_val;
        size_t i;
        union {
              u8 data[4];
              __le32 status_timeout;
        } u;

        for (i = 0; i < 4; i++)
        {
                memset(&cmd, 0, sizeof(struct systec_cmd_msg));
                cmd.size = SYSTEC_CMD_SIZE;
                cmd.cmd_data[0] = USBCAN_CMD_READEEPROM;
                cmd.cmd_data[1] = USBCAN_EEPROM_ADDR_STATUS_TIMEOUT + i;
                ret_val = systec_can_send_cmd(shared_res, &cmd);
                if (ret_val)
                        return ret_val;
                u.data[i] = cmd.cmd_data[2];
        }

        *status_timeout = le32_to_cpu(u.status_timeout);
        return 0;
}

static int systec_can_set_mode(struct net_device *netdev, enum can_mode mode)
{
        struct systec_can_chan *chan;

        chan = netdev_priv(netdev);

        systec_netdev_dbg(SYSTEC_DEBUG_DRIVER, netdev, "systec_can_set_mode: %d\n", (int)mode);

        switch (mode) {
        case CAN_MODE_START:
                /* schedule restart of the channel */
                chan->can.state = CAN_STATE_ERROR_ACTIVE;
                queue_work(cmd_wq, &chan->restart_work.work);

                break;

        default:
                return(-EOPNOTSUPP);
        }

        return 0;
}

static void systec_work_chan_restart(struct work_struct *work)
{
        struct systec_work *chan_work;
        struct systec_can_chan *chan;
        struct systec_cmd_msg cmd;
        int ret_val;

        chan_work = container_of(work, struct systec_work, work);
        chan = chan_work->chan;

        systec_netdev_dbg(SYSTEC_DEBUG_ERR_HANDLING, chan->netdev, "systec_work_chan_restart\n");

        /* reset channel */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_RESET;
        cmd.cmd_data[6] = chan->chan_no;
        ret_val = systec_can_send_cmd(chan->shared_res, &cmd);

        if (netif_queue_stopped(chan->netdev)) {
                netif_wake_queue(chan->netdev);
        }

        chan->restart_necessary = 0;
        return;
}

static void systec_unlink_urbs(struct systec_can_chan *chan)
{
        usb_kill_anchored_urbs(&chan->tx.tx_submitted);
        atomic_set(&chan->tx.active_tx_urbs, 0);
}

static int systec_can_send_cmd(struct systec_can_shared *shared_res, struct systec_cmd_msg *msg)
{
        int actual_length;
        int ret_stat;

        mutex_lock(&shared_res->cmd_pending);

        /* copy command to be sent */
        memcpy(&shared_res->tx_cmd_buffer[0], &msg->cmd_data[0], msg->size);

        systec_dev_dbg(SYSTEC_DEBUG_CMD, &shared_res->udev->dev, "%s\n", __func__);

        systec_dump_cmd_buf(shared_res, shared_res->tx_cmd_buffer);

        /* send command */
        ret_stat =  usb_bulk_msg(shared_res->udev,
                                usb_sndbulkpipe(shared_res->udev,
                                                shared_res->endpoint_addresses[SYSTEC_EP_MSG_OUT]),
                                &shared_res->tx_cmd_buffer[0],
                                msg->size,
                                &actual_length, SYSTEC_BULK_TIMEOUT);

        systec_dev_dbg(SYSTEC_DEBUG_CMD, &shared_res->udev->dev, "%s: %d, %d\n",
               __func__, actual_length, ret_stat);

        if (ret_stat < 0)
        {
                dev_err(&shared_res->udev->dev, "Sending command %d failed: %d\n", msg->cmd_data[0], ret_stat);
                goto unlock;
        }
        /* get command status  */
        ret_stat =  usb_bulk_msg(shared_res->udev,
                                usb_rcvbulkpipe(shared_res->udev,
                                                shared_res->endpoint_addresses[SYSTEC_EP_MSG_IN]),
                                &shared_res->rx_cmd_buffer[0],
                                msg->size,
                                &actual_length, SYSTEC_BULK_TIMEOUT);

        systec_dev_dbg(SYSTEC_DEBUG_CMD, &shared_res->udev->dev, "%s: %d, %d\n",
               __func__, actual_length, ret_stat);

        systec_dump_cmd_buf(shared_res, shared_res->rx_cmd_buffer);

        if (ret_stat < 0)
        {
                dev_err(&shared_res->udev->dev, "Receiving response to command %d failed: %d\n", msg->cmd_data[0], ret_stat);
        }

        memcpy(&msg->cmd_data[0], &shared_res->rx_cmd_buffer[0], actual_length);

unlock:
        mutex_unlock(&shared_res->cmd_pending);

        return ret_stat;
}

static void systec_dump_cmd_buf(struct systec_can_shared *shared_res, u8 *buffer)
{
        char print_buf[3 * 8 + 1];

        hex_dump_to_buffer(buffer, 8, 16, 1,
                   print_buf, sizeof(print_buf), false);

        systec_dev_dbg(SYSTEC_DEBUG_CMD_CB, &shared_res->udev->dev, "  cmd buf: %s\n", print_buf);
}

static void systec_dump_msg_buf(struct systec_can_shared *shared_res, u8 *buffer)
{
        char print_buf[3 * 16 + 1];

        hex_dump_to_buffer(buffer, 16, 16, 1,
                   print_buf, sizeof(print_buf), false);

        systec_dev_dbg(SYSTEC_DEBUG_DATA_CB, &shared_res->udev->dev, "  msg buf: %s\n", print_buf);
}

static void systec_dump_buf(struct systec_can_shared *shared_res, u8 *buffer, int size)
{
        char print_buf[3 * 16 + 1];

        hex_dump_to_buffer(buffer, min(size, 16), 16, 1,
                   print_buf, sizeof(print_buf), false);

        systec_dev_dbg(SYSTEC_DEBUG_STAT_CB, &shared_res->udev->dev, "  stat buf: %s\n", print_buf);
}

static ssize_t systec_can_sysfs_show_devicenr(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);

        if (!shared_res) {
                return -ENODEV;
        }
        else {
                return snprintf(buf, PAGE_SIZE, "%d\n", (int) shared_res->device_no);
        }
}

static ssize_t systec_can_sysfs_set_devicenr(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);
        struct systec_cmd_msg cmd;
        unsigned long devicenr;
        ssize_t ret_size;
        int ret_val;

        if (!shared_res) {
                return -ENODEV;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
        ret_val = kstrtoul(buf, 0, &devicenr);
#else
        ret_val = strict_strtoul(buf, 0, &devicenr);
#endif
        if (ret_val) {
                ret_size = ret_val;
                goto out;
        }

        if (devicenr > 254) {
                ret_size = -EINVAL;
                goto out;
        }

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));

        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_WRITEEEPROM;
        cmd.cmd_data[1] = USBCAN_EEPROM_DEVICENR;
        cmd.cmd_data[2] = (u8) devicenr;

        ret_val = systec_can_send_cmd(shared_res, &cmd);

        if (ret_val) {
                ret_size = ret_val;
                goto out;
        }

        shared_res->device_no = (u8) devicenr;
        ret_size = count;
out:
        return ret_size;
}

static DEVICE_ATTR(devicenr, S_IWUSR | S_IRUGO,
        systec_can_sysfs_show_devicenr, systec_can_sysfs_set_devicenr);

static ssize_t systec_can_sysfs_set_reset(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);
        struct systec_cmd_msg cmd;
        int ret_val;

        if (!shared_res) {
                return -ENODEV;
        }

        memset(&cmd, 0, sizeof(struct systec_cmd_msg));

        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_RESET_HW;
        cmd.cmd_data[1] = 1;

        ret_val = systec_can_send_cmd(shared_res, &cmd);

        if (!ret_val) {
                ret_val = count;
        }

        return ret_val;
}

static DEVICE_ATTR(reset, S_IWUSR,
        NULL, systec_can_sysfs_set_reset);

static ssize_t systec_can_sysfs_show_dual_channel(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);

        if (!shared_res) {
                return -ENODEV;
        }
        else {
                return snprintf(buf, PAGE_SIZE, "%d\n", systec_dual_chan_device(shared_res));
        }
}

static DEVICE_ATTR(dual_channel, S_IRUGO,
        systec_can_sysfs_show_dual_channel, NULL);

static ssize_t systec_can_sysfs_show_status_timeout(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);
        u32 status_timeout;
        int ret_val;

        if (!shared_res) {
                return -ENODEV;
        }
        else {
                ret_val = systec_status_timeout(shared_res, &status_timeout);
                if (ret_val)
                        return ret_val;
                else
                        return snprintf(buf, PAGE_SIZE, "%u\n", status_timeout);
        }
}

static ssize_t systec_can_sysfs_store_status_timeout(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct usb_interface *intf = to_usb_interface(dev);
        struct systec_can_shared *shared_res = usb_get_intfdata(intf);
        struct systec_cmd_msg cmd;
        int ret_val;
        unsigned long status_timeout;
        int old_timeout;
        size_t i;
        union {
              u8 data[4];
              __le32 status_timeout;
        } u;

        if (!shared_res) {
                return -ENODEV;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
        ret_val = kstrtoul(buf, 0, &status_timeout);
#else
        ret_val = strict_strtoul(buf, 0, &status_timeout);
#endif
        if (ret_val) {
                goto out;
        }


        ret_val = systec_status_timeout(shared_res, &old_timeout);
        if (ret_val)
        {
                goto out;
        }

        /* Check if set value is the same as already stored */
        if (old_timeout == status_timeout)
        {
                /* Do not write to EEPROM in that case */
                ret_val = count;
                goto out;
        }

        /* Update EEPROM if timeout is different */
        u.status_timeout = cpu_to_le32(status_timeout);
        for (i = 0; i < 4; i++)
        {
                memset(&cmd, 0, sizeof(struct systec_cmd_msg));
                cmd.size = SYSTEC_CMD_SIZE;
                cmd.cmd_data[0] = USBCAN_CMD_WRITEEEPROM;
                cmd.cmd_data[1] = USBCAN_EEPROM_ADDR_STATUS_TIMEOUT + i;
                cmd.cmd_data[2] = u.data[i];
                ret_val = systec_can_send_cmd(shared_res, &cmd);
                if (ret_val)
                        return ret_val;
        }
        ret_val = count;

out:
        return ret_val;
}

static DEVICE_ATTR(status_timeout, S_IRUGO | S_IWUSR,
        systec_can_sysfs_show_status_timeout, systec_can_sysfs_store_status_timeout);

static struct attribute *systec_can_device_sysfs_attrs[] = {
        &dev_attr_devicenr.attr,
        &dev_attr_reset.attr,
        &dev_attr_dual_channel.attr,
        &dev_attr_status_timeout.attr,
        NULL,
};

static struct attribute_group systec_can_device_sysfs_attr_group = {
        .attrs = systec_can_device_sysfs_attrs,
};

static ssize_t systec_can_sysfs_show_channel(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct net_device *netdev = to_net_dev(dev);
        struct systec_can_chan *chan = netdev_priv(netdev);

        return snprintf(buf, PAGE_SIZE, "%u\n", chan->chan_no);
}

static DEVICE_ATTR(channel, S_IRUGO,
        systec_can_sysfs_show_channel, NULL);

static ssize_t systec_can_sysfs_show_tx_timeout_ms(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct net_device *netdev = to_net_dev(dev);
        struct systec_can_chan *chan = netdev_priv(netdev);

        if (!systec_dual_chan_device(chan->shared_res))
        {
                return -ENOSYS;
        }
        return snprintf(buf, PAGE_SIZE, "%u\n", chan->tx_timeout_ms);
}

static ssize_t systec_can_sysfs_set_tx_timeout_ms(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct net_device *netdev = to_net_dev(dev);
        struct systec_can_chan *chan = netdev_priv(netdev);
        int ret_val;
        long unsigned int timeout;

        if (!systec_dual_chan_device(chan->shared_res))
        {
                ret_val = -ENOSYS;
                goto out;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
        ret_val = kstrtoul(buf, 0, &timeout);
#else
        ret_val = strict_strtoul(buf, 0, &timeout);
#endif
        if (ret_val)
        {
                goto out;
        }
        chan->tx_timeout_ms = timeout;

        ret_val = systec_setup_tx_timeout(chan);

        if (ret_val == 0)
                ret_val = count;
out:
        return ret_val;
}

static DEVICE_ATTR(tx_timeout_ms, S_IWUSR | S_IRUGO,
        systec_can_sysfs_show_tx_timeout_ms, systec_can_sysfs_set_tx_timeout_ms);

static struct attribute *systec_can_interface_sysfs_attrs[] = {
        &dev_attr_channel.attr,
        &dev_attr_tx_timeout_ms.attr,
        NULL,
};

static struct attribute_group systec_can_interface_sysfs_attr_group = {
        .attrs = systec_can_interface_sysfs_attrs,
};

static int systec_can_update_firmware(struct usb_interface *intf,
                            const void *fw_data,
                            size_t fw_size,
                            const struct firmware_header* fw_header)
{
        int ret_stat;
        struct usb_device *udev;
        struct device *dev = &intf->dev;
        const struct intel_hex_record *fw_record = fw_data;
        int download_size;
        int count;
        u8 *buffer;
        u32 *update_buf;

        udev = interface_to_usbdev(intf);

        /* start update */
        ret_stat = usb_control_msg(udev,
                                usb_sndctrlpipe (udev, 0),
                                USBCAN_VRREQ_START_UPDATE,
                                USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                0, 0, NULL, 0, SYSTEC_BULK_TIMEOUT);
        if (ret_stat < 0) {
                dev_err(dev, "could not prepare device for firmware upload (code %d)",
                                ret_stat);
                return(-ENODEV);
        }

        update_buf = devm_kmalloc(dev, 3 * sizeof(*update_buf), GFP_KERNEL);
        if (!update_buf)
            return -ENOMEM;

        buffer = kmalloc(sizeof(struct intel_hex_record), GFP_KERNEL);
        if (buffer == NULL) {
                dev_err(dev, "could not alloc buffer to upload firmware");
                return(-ENOMEM);
        }

        count = 0;
        download_size = 0;
        while (fw_record->type != 1 && download_size < fw_size) {
                memcpy(buffer, fw_record, sizeof(struct intel_hex_record));

                /* send record to CAN device */
                ret_stat = usb_control_msg(udev,
                                        usb_sndctrlpipe (udev, 0),
                                        USBCAN_VRREQ_WRITE_UPDATE,
                                        USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        0, 0, buffer,
                                        sizeof(fw_record->type) + sizeof(fw_record->length)
                                        + sizeof(fw_record->address) + fw_record->length,
                                        SYSTEC_BULK_TIMEOUT);

                if (ret_stat < 0) {
                        dev_err(dev, "firmware update failed at address: 0x%x, code: %d",
                                fw_record->address, ret_stat);

                        kfree(buffer);
                        return(-ENODEV);
                }

                fw_record++;
                count++;
                download_size += sizeof(struct intel_hex_record);
        }

        kfree(buffer);

        if (download_size >= fw_size) {
                /* should never happen (intel hex record eof not found) */
                dev_err(dev, "intel hex record eof not found");
                return(-ENODEV);
        }

        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "%d firmware records uploaded", count);

        /* check update status */
        ret_stat = usb_control_msg(udev,
                                usb_rcvctrlpipe (udev, 0),
                                USBCAN_VRREQ_CHECK_UPDATE,
                                USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                0, 0, update_buf, 3 * sizeof(*update_buf),
                                SYSTEC_BULK_TIMEOUT);

        if (ret_stat != 3 * sizeof(*update_buf)) {
                dev_warn(dev, "could not check update result (code %d)",
                        ret_stat);
                return(-ENODEV);
        }

        /* stop update */
        if (le32_to_cpu(update_buf[0]) == 0) {
                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "stopping update");
                ret_stat = usb_control_msg(udev,
                                        usb_sndctrlpipe (udev, 0),
                                        USBCAN_VRREQ_STOPP_UPDATE,
                                        USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        0, 0, NULL, 0, SYSTEC_BULK_TIMEOUT);

                if (ret_stat < 0) {
                        dev_err(dev, "could not stop update (code %d)",
                                ret_stat);
                        return(-ENODEV);
                }

                /* write firmware to flash */
                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "writing firmware to flash");
                update_buf[0] = fw_header->fw_address_le;
                update_buf[1] = fw_header->buffer_size_le;
                update_buf[2] = 0;

                ret_stat = usb_control_msg(udev,
                                        usb_sndctrlpipe (udev, 0),
                                        USBCAN_VRREQ_WRITE_FLASH,
                                        USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        0, 0, update_buf, 3 * sizeof(*update_buf),
                                        SYSTEC_BULK_TIMEOUT);

                if (ret_stat < 0) {
                        dev_err(dev, "could not write firmware to flash (code %d)",
                                ret_stat);
                        return(-ENODEV);
                }

        }
        else {
                dev_err(dev, "update failed (device returned 0x%x 0x%x)",
                        le32_to_cpu(update_buf[0]), le32_to_cpu(update_buf[1]));
                return(-ENODEV);
        }

        return (0);
}

static int systec_can_main_probe_boot(struct usb_interface *intf,
                            const struct usb_device_id *id)
{
        int ret_stat;
        __le32 *buf;
        u32 current_version;
        struct usb_device *udev;
        const struct firmware *fw;
        struct device *dev = &intf->dev;
        __le32 *update_buf;
        const char *fw_name_template = FIRMWARE_NAME;
        const char *bl_fw_name_template = BOOTLOADER_NAME;
        char *fw_name;
        const struct firmware_header *header;
        u32 fw_file_version;

        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "systec_can_main_probe_boot\n");

        /* indicate that CAN device is in boot mode */
        usb_set_intfdata(intf, NULL);

        udev = interface_to_usbdev(intf);

        current_version = 0;

        buf = devm_kmalloc(dev, sizeof(*buf), GFP_KERNEL);
        if (!buf)
            return -ENOMEM;

        update_buf = devm_kmalloc(dev, 3 * sizeof(*update_buf), GFP_KERNEL);
        if (!update_buf)
            return -ENOMEM;

        /* read bootloader version from device */
        ret_stat = usb_control_msg (udev,
                                    usb_rcvctrlpipe (udev, 0),
                                    USBCAN_VRREQ_READ_VERSION,
                                    USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                    0,
                                    0,
                                    buf,
                                    sizeof (*buf),
                                    SYSTEC_BULK_TIMEOUT);

        if (ret_stat != sizeof (*buf)) {
                dev_err(dev, "could not read bootloader version from device (code %d)",
                        ret_stat);
                dev_err(dev, "It seems that no bootloader is present on this device. Contact vendor for support.");
                return(-ENODEV);
        }

        current_version = le32_to_cpup(buf);

        dev_info(dev, "device bootloader version: %d.%02d r%d",
                GETPC_MAJOR_VER(current_version),GETPC_MINOR_VER(current_version),
                GETPC_RELEASE_VER(current_version));

        /* read bootloader version from bootloader image */
        fw_name = kmalloc(strlen(bl_fw_name_template) + 16, GFP_KERNEL);
        if (!fw_name) {
                dev_err(dev, "could not alloc buffer to create bootloader name");
                return(-ENOMEM);
        }

        sprintf(fw_name, bl_fw_name_template, (u16)id->idProduct);

        dev_info(dev, "request firmware image: %s", fw_name);

        /* request bootloader */
        if (request_firmware(&fw, fw_name, dev) == 0) {

                kfree(fw_name);

                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "request_firmware: %zd bytes", fw->size);
        }
        else {
                dev_err(dev, "firmware request failed (%s)", fw_name);
                kfree(fw_name);

                return(-ENODEV);
        }

        header = (struct firmware_header *)fw->data;

        if (header->type != SYSTEC_TYPE_BL)
        {
                dev_err(dev, "wrong bootloader type 0x%02x for idProduct 0x%04x", header->type, id->idProduct);

                ret_stat = -EINVAL;
                goto free_fw;
        }

        if (le16_to_cpu(header->product_id_le) != id->idProduct)
        {
                dev_err(dev, "wrong bootloader product id %.4x (device product id is %.4x)",
                        le16_to_cpu(header->product_id_le), id->idProduct);

                ret_stat = -EINVAL;
                goto free_fw;
        }

        fw_file_version = FW_VERSION(header->fw_major, header->fw_minor, le16_to_cpu(header->fw_release_le));

        dev_info(dev, "version in bootloader file: %d.%02d r%d",
                GETPC_MAJOR_VER(fw_file_version),GETPC_MINOR_VER(fw_file_version),
                GETPC_RELEASE_VER(fw_file_version));

        ret_stat = 0;
        if ((GETPC_MAJOR_VER(current_version) < GETPC_MAJOR_VER(fw_file_version)) ||
                (GETPC_MINOR_VER(current_version) < GETPC_MINOR_VER(fw_file_version)) ||
                (GETPC_RELEASE_VER(current_version) < GETPC_RELEASE_VER(fw_file_version)))
        {
                /* Need to update bootloader */
                ret_stat = systec_can_update_firmware(intf, fw->data + sizeof(*header), fw->size - sizeof(*header), header);

                release_firmware(fw);

                /*
                 * Finish probe here sucessfully.
                 * the device will disconnect itself after update has finished
                 */
                return 0;
        }

        release_firmware(fw);

        if (ret_stat) {
                dev_err(dev, "bootloader update failed (code %d)", ret_stat);
                return ret_stat;
        }

        /* get required firmware version based on product id */
        current_version = 0;

        /* read firmware version from device */
        ret_stat = usb_control_msg (udev,
                                    usb_rcvctrlpipe (udev, 0),
                                    USBCAN_VRREQ_READ_FW_VERSION,
                                    USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                    0,
                                    0,
                                    buf,
                                    sizeof (*buf),
                                    SYSTEC_BULK_TIMEOUT);

        if (ret_stat != sizeof (*buf)) {
                dev_err(dev, "could not read firmware version from device (code %d)",
                        ret_stat);
                dev_err(dev, "It seems that no bootloader is present on this device. Contact vendor for support.");
                return(-ENODEV);
        }

        current_version = le32_to_cpup(buf);

        /* read firmware version from firmware image */
        fw_name = kmalloc(strlen(fw_name_template) + 16, GFP_KERNEL);
        if (!fw_name) {
                dev_err(dev, "could not alloc buffer to create firmware name");
                return(-ENOMEM);
        }

        sprintf(fw_name, fw_name_template, (u16)id->idProduct);

        dev_info(dev, "request firmware image: %s", fw_name);

        /* request firmware */
        if (request_firmware(&fw, fw_name, dev) == 0) {

                kfree(fw_name);

                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "request_firmware: %zd bytes", fw->size);
        }
        else {
                dev_err(dev, "firmware request failed (%s)", fw_name);
                kfree(fw_name);

                return(-ENODEV);
        }

        header = (struct firmware_header *)fw->data;

        if (header->type != SYSTEC_TYPE_FW)
        {
                dev_err(dev, "wrong firmware type 0x%02x for idProduct 0x%04x", header->type, id->idProduct);

                ret_stat = -EINVAL;
                goto free_fw;
        }

        if (le16_to_cpu(header->product_id_le) != id->idProduct)
        {
                dev_err(dev, "wrong firmware product id %.4x (device product id is %.4x)",
                        le16_to_cpu(header->product_id_le), id->idProduct);

                ret_stat = -EINVAL;
                goto free_fw;
        }

        fw_file_version = FW_VERSION(header->fw_major, header->fw_minor, le16_to_cpu(header->fw_release_le));

        dev_info(dev, "device firmware version: %d.%02d r%d",
                GETPC_MAJOR_VER(current_version),GETPC_MINOR_VER(current_version),
                GETPC_RELEASE_VER(current_version));
        dev_info(dev, "version in firmware file: %d.%02d r%d",
                GETPC_MAJOR_VER(fw_file_version),GETPC_MINOR_VER(fw_file_version),
                GETPC_RELEASE_VER(fw_file_version));

        if (current_version == fw_file_version) {

                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "   firmware of device is up to date");

                /* ceck crc of firmware */
                update_buf[0] = header->fw_address_le;
                update_buf[1] = header->buffer_size_le;

                ret_stat = usb_control_msg(udev,
                                        usb_sndctrlpipe (udev, 0),
                                        USBCAN_VRREQ_CHECK_CRC,
                                        USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        0, 0, update_buf, 2 * sizeof(*update_buf),
                                        SYSTEC_BULK_TIMEOUT);

                if (ret_stat < 0) {
                        dev_err(dev, "could not run crc check (code %d)",
                                ret_stat);
                        ret_stat =  -ENODEV;
                        goto free_fw;
                }

                /* get result of crc check */
                memset(update_buf, 0, 3 * sizeof(*update_buf));
                ret_stat = usb_control_msg(udev,
                                        usb_rcvctrlpipe (udev, 0),
                                        USBCAN_VRREQ_GET_CRC_RESULT,
                                        USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                        0, 0, update_buf, 3 * sizeof(*update_buf),
                                        SYSTEC_BULK_TIMEOUT);

                if (ret_stat != 3 * sizeof(*update_buf)) {
                        dev_err(dev, "could not get result of crc check (code %d)",
                                ret_stat);
                        ret_stat =  -ENODEV;
                        goto free_fw;
                }

                if (le32_to_cpu(update_buf[0]) == 0) {

                        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, dev, "   firmware crc check passed");

                        /* boot CAN device via reconnect */
                        ret_stat = usb_control_msg (udev,
                                                usb_sndctrlpipe (udev, 0),
                                                USBCAN_VRREQ_RECONNECT,
                                                USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                                                0, 0, NULL, 0, SYSTEC_BULK_TIMEOUT);

                        goto free_fw;
                }

                dev_warn(dev, "firmware crc check failed (ErrorCode=0x%08X, CryptCrc32=0x%08X, StartMode=0x%08X)",
                        le32_to_cpu(update_buf[0]),
                        le32_to_cpu(update_buf[1]),
                        le32_to_cpu(update_buf[2]));

                /* continue with download of firmware, because CRC check failed */
        }

        ret_stat = systec_can_update_firmware(intf, fw->data + sizeof(*header), fw->size - sizeof(*header), header);

free_fw:
        release_firmware(fw);

        devm_kfree(dev, buf);

        return ret_stat;
}


static void systec_can_close_channel(struct usb_interface *intf,
                            struct systec_can_shared *shared_res,
                            int chan_no)
{
    if (shared_res->channels[chan_no]) {

            dev_info(&intf->dev, "unregister device: %s (channel %d)\n", shared_res->channels[chan_no]->netdev->name, chan_no);
            unregister_netdev(shared_res->channels[chan_no]->netdev);

    }
}


static void systec_can_free_channel(struct usb_interface *intf,
                            struct systec_can_shared *shared_res,
                            int chan_no)
{
    if (shared_res->channels[chan_no]) {

            dev_info(&intf->dev, "free device: %s (channel %d)\n", shared_res->channels[chan_no]->netdev->name, chan_no);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
            sysfs_remove_group(&shared_res->channels[chan_no]->netdev->dev.kobj, &systec_can_interface_sysfs_attr_group);
#endif
            free_candev(shared_res->channels[chan_no]->netdev);
            shared_res->channels[chan_no] = NULL;
    }
}


static int systec_can_alloc_channel(struct usb_interface *intf,
                            struct systec_can_shared *shared_res,
                            int chan_no)
{
        int ret_stat = -ENOMEM;
        struct net_device *netdev;
        struct systec_can_chan *chan;
        int i;

        /* prepare channel 1 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
        netdev = alloc_candev(sizeof(struct systec_can_chan), TX_ECHO_SKB_MAX);
#else
        netdev = alloc_candev(sizeof(struct systec_can_chan));
#endif
        if (!netdev) {
                dev_err(&intf->dev, "Couldn't alloc candev for channel %d\n",
                        chan_no);

                return(-ENOMEM);
        }

        /* get private part */
        chan = netdev_priv(netdev);

        shared_res->channels[chan_no] = chan;

        chan->netdev = netdev;
        chan->chan_no = chan_no;
        chan->mod_status_int = 0;

        /* init CAN parameters */
        chan->can.state = CAN_STATE_STOPPED;
        chan->can.clock.freq = SYSTEC_DEVICE_CLOCK;
        chan->can.bittiming_const = &systec_can_bittiming_const;
        chan->can.do_set_bittiming = systec_can_set_bittiming;
        chan->can.do_set_mode = systec_can_set_mode;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        chan->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
                                       CAN_CTRLMODE_LISTENONLY;
#endif

        netdev->netdev_ops = &systec_can_netdev_ops;

        /* support local echo */
        netdev->flags |= IFF_ECHO;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
        netdev->sysfs_groups[0] = &systec_can_interface_sysfs_attr_group;
#endif

        init_usb_anchor(&chan->tx.tx_submitted);
        atomic_set(&chan->tx.active_tx_urbs, 0);
        atomic_set(&chan->tx.pending, 0);

        SET_NETDEV_DEV(netdev, &intf->dev);

        /* device needs access to shared ressources */
        chan->shared_res = shared_res;

        /* init work to reset channel (bus-off) */
        INIT_WORK(&chan->restart_work.work,
                systec_work_chan_restart);
        chan->restart_work.chan = chan;

        /* create mac address out of serial number, device number, channel number */
        for (i = 0; i < 4; i++) {
                netdev->dev_addr[i] = shared_res->serial_no[3 - i];
        }
        netdev->dev_addr[4] = shared_res->device_no;
        netdev->dev_addr[5] = chan_no;
        netdev->addr_len = 6;

        ret_stat = register_candev(netdev);
        if (ret_stat) {
                dev_err(&intf->dev,
                        "couldn't register CAN device for channel %d: %d\n", chan_no, ret_stat);
                goto fail;
        }

        dev_info(&intf->dev, "registered device: %s (channel %d), %pM\n",
                netdev->name, chan->chan_no, netdev->dev_addr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
        devm_can_led_init(netdev);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
        // register sysfs attributes
        ret_stat = sysfs_create_group(&netdev->dev.kobj, &systec_can_interface_sysfs_attr_group);
        if (ret_stat)
        {
                netdev_err(netdev, "failed to create sysfs device attributes: %d", ret_stat);
                goto fail;
        }
#endif
        return (0);

fail:
        free_candev(netdev);
        return ret_stat;
}


static int systec_can_main_probe(struct usb_interface *intf,
                            const struct usb_device_id *id)
{
        struct systec_can_shared *shared_res;
        struct usb_host_interface       *usb_host_intf;
        struct usb_endpoint_descriptor  *usb_endp_desc;
        struct systec_cmd_msg cmd;
        int ret_stat = -ENOMEM;
        int i;

        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "systec_can_main_probe\n");

        if (id->idProduct != 0x1101 && id->idProduct != 0x1181) {
                /* boot device */
                return (systec_can_main_probe_boot(intf, id));
        }

        /* alloc ressource shared between both channels */
        shared_res = kzalloc(sizeof(struct systec_can_shared), GFP_KERNEL);
        if (!shared_res) {
                dev_err(&intf->dev, "couldn't alloc shared ressource\n");

                return (-ENOMEM);
        }

        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "  shared_res: %p\n", shared_res);

        mutex_init(&shared_res->cmd_pending);

        shared_res->udev = interface_to_usbdev(intf);

        usb_host_intf = &intf->altsetting[0];
        if (intf->num_altsetting < 1 || usb_host_intf->desc.bNumEndpoints < 5) {
                dev_err(&intf->dev, "error in usb-endpoints\n");
                kfree(shared_res);
                return (-ENODEV);
        }

        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "  num_altsetting: %d\n", intf->num_altsetting);
        systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "  bNumEndpoints: %d\n", usb_host_intf->desc.bNumEndpoints);

        /* get endpoints for fast access */
        usb_endp_desc = &usb_host_intf->endpoint[SYSTEC_EP_DATA_OUT].desc;
        if ((usb_endp_desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == 0
            && (usb_endp_desc->bmAttributes  & USB_ENDPOINT_XFER_BULK) != 0) {
                shared_res->endpoint_addresses[SYSTEC_EP_DATA_OUT] = usb_endp_desc->bEndpointAddress;
        }
        else {
                dev_err(&intf->dev, "missing data-out endpoint\n");
                kfree(shared_res);
                return (-ENODEV);
        }

        usb_endp_desc = &usb_host_intf->endpoint[SYSTEC_EP_DATA_IN].desc;
        if ((usb_endp_desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) != 0
            && (usb_endp_desc->bmAttributes  & USB_ENDPOINT_XFER_BULK) != 0) {
                shared_res->endpoint_addresses[SYSTEC_EP_DATA_IN] = usb_endp_desc->bEndpointAddress;
        }
        else {
                dev_err(&intf->dev, "missing data-in endpoint\n");
                kfree(shared_res);
                return (-ENODEV);
        }


        usb_endp_desc = &usb_host_intf->endpoint[SYSTEC_EP_MSG_OUT].desc;
        if ((usb_endp_desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == 0
            && (usb_endp_desc->bmAttributes  & USB_ENDPOINT_XFER_BULK) != 0) {
                shared_res->endpoint_addresses[SYSTEC_EP_MSG_OUT] = usb_endp_desc->bEndpointAddress;
        }
        else {
                dev_err(&intf->dev, "missing message-out endpoint\n");
                kfree(shared_res);
                return (-ENODEV);
        }

        usb_endp_desc = &usb_host_intf->endpoint[SYSTEC_EP_MSG_IN].desc;
        if ((usb_endp_desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) != 0
            && (usb_endp_desc->bmAttributes  & USB_ENDPOINT_XFER_BULK) != 0) {
                shared_res->endpoint_addresses[SYSTEC_EP_MSG_IN] = usb_endp_desc->bEndpointAddress;
        }
        else {
                dev_err(&intf->dev, "missing message-in endpoint\n");
                kfree(shared_res);
                return (-ENODEV);
        }

        usb_endp_desc = &usb_host_intf->endpoint[SYSTEC_EP_STAT_IN].desc;
        if ((usb_endp_desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) != 0
            && (usb_endp_desc->bmAttributes  & USB_ENDPOINT_XFER_BULK) != 0) {
                shared_res->endpoint_addresses[SYSTEC_EP_STAT_IN] = usb_endp_desc->bEndpointAddress;
        }
        else {
                dev_err(&intf->dev, "missing status endpoint\n");
                kfree(shared_res);
                return (-ENODEV);
        }

        for (i = 0; i < 5; i++)
        {
                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "    endpoint %d: %d\n", i, shared_res->endpoint_addresses[i]);
        }

        /* init shared ressources */
        init_usb_anchor(&shared_res->rx_submitted);
        shared_res->intr_urb = NULL;

        shared_res->intr_in_buffer = kzalloc(INTR_IN_BUFFER_SIZE, GFP_KERNEL);
        if (!shared_res->intr_in_buffer) {
                dev_err(&intf->dev, "Couldn't alloc Intr buffer\n");
                kfree(shared_res);
                return (ret_stat);
        }

        shared_res->tx_cmd_buffer = kzalloc(sizeof(struct systec_cmd_msg), GFP_KERNEL);
        if (!shared_res->tx_cmd_buffer) {
                dev_err(&intf->dev, "Couldn't alloc Tx cmd buffer\n");
                kfree(shared_res->intr_in_buffer);
                kfree(shared_res);
                return (ret_stat);
        }

        shared_res->rx_cmd_buffer = kzalloc(sizeof(struct systec_cmd_msg), GFP_KERNEL);
        if (!shared_res->rx_cmd_buffer) {
                dev_err(&intf->dev, "Couldn't alloc Rx cmd buffer\n");
                kfree(shared_res->tx_cmd_buffer);
                kfree(shared_res->intr_in_buffer);
                kfree(shared_res);
                return (ret_stat);
        }

        systec_get_serial_no(shared_res);

        /* stop channel 0 */
        memset(&cmd, 0, sizeof(struct systec_cmd_msg));
        cmd.size = SYSTEC_CMD_SIZE;
        cmd.cmd_data[0] = USBCAN_CMD_SHUTDOWN;
        cmd.cmd_data[6] = 0;
        ret_stat = systec_can_send_cmd(shared_res, &cmd);
        if (ret_stat != 0) {

                kfree(shared_res->rx_cmd_buffer);
                kfree(shared_res->tx_cmd_buffer);
                kfree(shared_res->intr_in_buffer);
                kfree(shared_res);
                return (ret_stat);
        }

        /* alloc channel 0 */
        ret_stat = systec_can_alloc_channel(intf, shared_res, 0);
        if (ret_stat != 0) {

                kfree(shared_res->rx_cmd_buffer);
                kfree(shared_res->tx_cmd_buffer);
                kfree(shared_res->intr_in_buffer);
                kfree(shared_res);
                return (ret_stat);
        }

        if (systec_dual_chan_device(shared_res)) {

                systec_dev_dbg(SYSTEC_DEBUG_DRIVER, &intf->dev, "    dual channel device found\n");

                /* stop channel */
                memset(&cmd, 0, sizeof(struct systec_cmd_msg));
                cmd.size = SYSTEC_CMD_SIZE;
                cmd.cmd_data[0] = USBCAN_CMD_SHUTDOWN;
                cmd.cmd_data[6] = 1;
                ret_stat = systec_can_send_cmd(shared_res, &cmd);
                if (ret_stat != 0) {

                        kfree(shared_res->rx_cmd_buffer);
                        kfree(shared_res->tx_cmd_buffer);
                        kfree(shared_res->intr_in_buffer);
                        kfree(shared_res);
                        return (ret_stat);
                }

                /* alloc channel 1 */
                ret_stat = systec_can_alloc_channel(intf, shared_res, 1);
                if (ret_stat != 0) {

                        systec_can_close_channel(intf, shared_res, 0);
                        systec_can_free_channel(intf, shared_res, 0);
                        kfree(shared_res->rx_cmd_buffer);
                        kfree(shared_res->tx_cmd_buffer);
                        kfree(shared_res->intr_in_buffer);
                        kfree(shared_res);
                        return (ret_stat);
                }
        }

        /* create shared sysfs attributes */
        ret_stat = sysfs_create_group(&intf->dev.kobj, &systec_can_device_sysfs_attr_group);
        if (ret_stat != 0) {

                systec_can_close_channel(intf, shared_res, 1);
                systec_can_close_channel(intf, shared_res, 0);
                systec_can_free_channel(intf, shared_res, 1);
                systec_can_free_channel(intf, shared_res, 0);
                kfree(shared_res->rx_cmd_buffer);
                kfree(shared_res->tx_cmd_buffer);
                kfree(shared_res->intr_in_buffer);
                kfree(shared_res);
                return (ret_stat);
        }

        /* attach shared ressources to USB interface */
        usb_set_intfdata(intf, shared_res);

        return 0;
}

static void systec_can_main_disconnect(struct usb_interface *intf)
{
        struct systec_can_shared *shared_res;
        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "systec_can_main_disconnect\n");

        shared_res = usb_get_intfdata(intf);
        if (!shared_res) {
                systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "disconnect during boot\n");

                /* shared_res == NULL indicates a disconnect during boot */
                return;
        }

        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "  shared_res: %p\n", shared_res);

        sysfs_remove_group(&intf->dev.kobj, &systec_can_device_sysfs_attr_group);
        usb_set_intfdata(intf, NULL);

        systec_can_close_channel(intf, shared_res, 1);
        systec_can_close_channel(intf, shared_res, 0);

        systec_can_free_channel(intf, shared_res, 1);
        systec_can_free_channel(intf, shared_res, 0);

        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "  kfree (intr_in_buffer)\n");

        kfree(shared_res->intr_in_buffer);

        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "  kfree (tx_cmd_buffer)\n");

        kfree(shared_res->tx_cmd_buffer);

        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "  kfree (rx_cmd_buffer)\n");

        kfree(shared_res->rx_cmd_buffer);

        systec_dev_dbg(SYSTEC_DEBUG_USB_DISCONNECT, &intf->dev, "  kfree (shared_res)\n");

        kfree(shared_res);

}


static int __init systec_can_main_init(void)
{
        int err;

        systec_dbg(SYSTEC_DEBUG_DRIVER, "systec_can driver loaded\n");


        if (!cmd_wq) {
                /* create the work queue to handle commands */
                cmd_wq = create_workqueue("systec_wq");
                if (!cmd_wq) {

                        pr_err("    error: could not create work queue\n");
/*
todo: proper return value
*/
                        return(-1);
                }
        }

        /* register this driver with the USB subsystem */
        err = usb_register(&systec_can_driver);

        if (err) {
                destroy_workqueue(cmd_wq);

                pr_err("usb_register failed. Error number %d\n", err);
                return err;
        }

        return 0;
}


static void __exit systec_can_main_exit(void)
{
        systec_dbg(SYSTEC_DEBUG_DRIVER, "systec_can_main_exit\n");

        destroy_workqueue(cmd_wq);

        /* deregister this driver with the USB subsystem */
        usb_deregister(&systec_can_driver);
}

module_init(systec_can_main_init);
module_exit(systec_can_main_exit);

MODULE_AUTHOR("Heino Gutschmidt <heino.gutschmidt@managedhosting.de>, "
              "Daniel Krueger <daniel.krueger@systec-electronic.com>,"
              "Alexander Stein <alexander.stein@systec-electronic.com>");
MODULE_DESCRIPTION(SYSTEC_MODULE_DESC);
MODULE_VERSION(SYSTEC_MODULE_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_FIRMWARE(FIRMWARE_FILE(1103));
MODULE_FIRMWARE(FIRMWARE_FILE(1104));
MODULE_FIRMWARE(FIRMWARE_FILE(1105));
MODULE_FIRMWARE(FIRMWARE_FILE(1121));
MODULE_FIRMWARE(FIRMWARE_FILE(1122));
MODULE_FIRMWARE(FIRMWARE_FILE(1145));
module_param(debug,  uint, 0644);
MODULE_PARM_DESC(debug, "Bitmask for individual debug output.");
module_param(hw_txecho,  bool, 0644);
MODULE_PARM_DESC(hw_txecho, "Use txecho from hardware rather than USB acknowledge.");
