#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#define _BSD_SOURCE
#include <endian.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "debug.h"

/*****************************************************************************/

#define ARRAY_SIZE(a)                   (sizeof(a) / sizeof(a[0]))

#define REG_STATUS_ITEM(reg)            { 0, reg, #reg }
#define SUBCMD_STATUS_ITEM(reg)         { 1, reg, #reg }

/*****************************************************************************/

#define BQ27X00_REG_CNTL                0x00 /* Control */
#define BQ27X00_REG_AR                  0x02 /* AtRate */
#define BQ27X00_REG_ARTTE               0x04 /* AtRateTimeToEmpty */
#define BQ27X00_REG_TEMP                0x06 /* Temperature */
#define BQ27X00_REG_VOLT                0x08 /* Voltage */
#define BQ27X00_REG_FLAGS               0x0A /* Flags */
#define BQ27X00_REG_NAC                 0x0C /* NominalAvailableCapacity */
#define BQ27X00_REG_FAC                 0x0E /* FullAvailableCapacity */
#define BQ27X00_REG_RM                  0x10 /* RemainingCapacity */
#define BQ27X00_REG_FCC                 0x12 /* FullChargeCapacity */
#define BQ27X00_REG_AI                  0x14 /* AverageCurrent */
#define BQ27X00_REG_TTE                 0x16 /* Time To Empty */
#define BQ27X00_REG_TTF                 0x18 /* StandbyCurrent */
#define BQ27X00_REG_STTE                0x1A /* StandbyTimeToEmpty */
#define BQ27X00_REG_SOH                 0x1C /* StateOfHealth */
#define BQ27X00_REG_CC                  0x1E /* CycleCount */
#define BQ27X00_REG_SOC                 0x20 /* StateOfCharge */
#define BQ27X00_REG_IC                  0x22 /* InstantaneousCurrent */
#define BQ27X00_REG_INTTEMP             0x28 /* InternalTemperature */
#define BQ27X00_REG_RSCALE              0x2A /* ResistanceScale */
#define BQ27X00_REG_OP_CONFIG           0x2C /* OperationConfiguration */
#define BQ27X00_REG_DCAP                0x2E /* DesignCapacity */
#define BQ27X00_REG_UFRM                0x6C /* UnfilteredRM */
#define BQ27X00_REG_FRM                 0x6E /* FilteredRM */
#define BQ27X00_REG_UFFCC               0x70 /* UnfilteredFCC */
#define BQ27X00_REG_FFCC                0x72 /* FilteredFCC */
#define BQ27X00_REG_UFSOC               0x74 /* TrueSOC */

#if 1
#define BQ27X00_REG_TTECP               0x26 /* TTEatConstantPower */
#define BQ27X00_REG_LMD                 0x12 /* Last measured discharge: FullChargeCapacity */
#define BQ27X00_REG_AE                  0x22 /* Available energy */
#define BQ27x00_POWER_AVG               0x24 /* Average Power */
#endif

enum {
    SUBCMD_CONTROL_STATUS = 0x0000,
    SUBCMD_DEVICE_TYPE = 0x0001,
    SUBCMD_FW_VERSION = 0x0002,
    SUBCMD_PREV_MACWRITE = 0x0007,
    SUBCMD_CHEM_ID = 0x0008,
    SUBCMD_OCV_CMD = 0x000C,
    SUBCMD_BAT_INSERT = 0x000D,
    SUBCMD_BAT_REMOVE = 0x000E,
    SUBCMD_SET_HIBERNATE = 0x0011,
    SUBCMD_CLEAR_HIBERNATE = 0x0012,
    SUBCMD_SET_SNOOZE = 0x0013,
    SUBCMD_CLEAR_SNOOZE = 0x0014,
    SUBCMD_DF_VERSION = 0x001F,
    SUBCMD_SEALED = 0x0020,
    SUBCMD_IT_ENABLE = 0x0021,
    SUBCMD_RESET = 0x0041,
};

/*****************************************************************************/

struct bq_device {
    int fd;
    int slave;
    int classid;
    int blockid;
    unsigned char block[32];
    unsigned char checksum;
};

struct bq_status_item {
    int is_subcmd;
    unsigned int addr;
    const char *name;
};

static struct bq_status_item show_stats[] = {
    SUBCMD_STATUS_ITEM(SUBCMD_CONTROL_STATUS),
    SUBCMD_STATUS_ITEM(SUBCMD_DEVICE_TYPE),
    SUBCMD_STATUS_ITEM(SUBCMD_FW_VERSION),
    SUBCMD_STATUS_ITEM(SUBCMD_PREV_MACWRITE),
    SUBCMD_STATUS_ITEM(SUBCMD_CHEM_ID),
    SUBCMD_STATUS_ITEM(SUBCMD_OCV_CMD),
    SUBCMD_STATUS_ITEM(SUBCMD_BAT_INSERT),
    SUBCMD_STATUS_ITEM(SUBCMD_BAT_REMOVE),
    SUBCMD_STATUS_ITEM(SUBCMD_SET_HIBERNATE),
    SUBCMD_STATUS_ITEM(SUBCMD_CLEAR_HIBERNATE),
    SUBCMD_STATUS_ITEM(SUBCMD_SET_SNOOZE),
    SUBCMD_STATUS_ITEM(SUBCMD_CLEAR_SNOOZE),
    SUBCMD_STATUS_ITEM(SUBCMD_DF_VERSION),
    SUBCMD_STATUS_ITEM(SUBCMD_SEALED),
    SUBCMD_STATUS_ITEM(SUBCMD_IT_ENABLE),
    SUBCMD_STATUS_ITEM(SUBCMD_RESET),

    REG_STATUS_ITEM(BQ27X00_REG_CNTL),
    REG_STATUS_ITEM(BQ27X00_REG_AR),
    REG_STATUS_ITEM(BQ27X00_REG_ARTTE),
    REG_STATUS_ITEM(BQ27X00_REG_TEMP),
    REG_STATUS_ITEM(BQ27X00_REG_VOLT),
    REG_STATUS_ITEM(BQ27X00_REG_FLAGS),
    REG_STATUS_ITEM(BQ27X00_REG_NAC),
    REG_STATUS_ITEM(BQ27X00_REG_FAC),
    REG_STATUS_ITEM(BQ27X00_REG_RM),
    REG_STATUS_ITEM(BQ27X00_REG_FCC),
    REG_STATUS_ITEM(BQ27X00_REG_AI),
    REG_STATUS_ITEM(BQ27X00_REG_TTE),
    REG_STATUS_ITEM(BQ27X00_REG_TTF),
    REG_STATUS_ITEM(BQ27X00_REG_STTE),
    REG_STATUS_ITEM(BQ27X00_REG_SOH),
    REG_STATUS_ITEM(BQ27X00_REG_CC),
    REG_STATUS_ITEM(BQ27X00_REG_SOC),
    REG_STATUS_ITEM(BQ27X00_REG_IC),
    REG_STATUS_ITEM(BQ27X00_REG_INTTEMP),
    REG_STATUS_ITEM(BQ27X00_REG_RSCALE),
    REG_STATUS_ITEM(BQ27X00_REG_OP_CONFIG),
    REG_STATUS_ITEM(BQ27X00_REG_DCAP),
    REG_STATUS_ITEM(BQ27X00_REG_UFRM),
    REG_STATUS_ITEM(BQ27X00_REG_FRM),
    REG_STATUS_ITEM(BQ27X00_REG_UFFCC),
    REG_STATUS_ITEM(BQ27X00_REG_FFCC),
    REG_STATUS_ITEM(BQ27X00_REG_UFSOC),
    REG_STATUS_ITEM(BQ27X00_REG_TTECP),
    REG_STATUS_ITEM(BQ27X00_REG_LMD),
    REG_STATUS_ITEM(BQ27X00_REG_AE),
    REG_STATUS_ITEM(BQ27x00_POWER_AVG),
};

/*****************************************************************************/

static inline void setU1(void *ptr, unsigned char value)
{
    unsigned char *p = ptr;
    p[0] = value;
}

static inline void setU2(void *ptr, unsigned short value)
{
    unsigned char *p = ptr;
    p[0] = (value >> 8) & 0xff;
    p[1] = value & 0xff;
}

static inline void setU4(void *ptr, unsigned int value)
{
    unsigned char *p = ptr;
    p[0] = (value >> 24) & 0xff;
    p[1] = (value >> 16) & 0xff;
    p[2] = (value >> 8) & 0xff;
    p[3] = value & 0xff;
}

/*****************************************************************************/

static struct bq_device *openDevice(int busnum, int slave)
{
    char path[64];
    struct bq_device *dev;

    dev = calloc(1, sizeof(*dev));
    ASSERT(dev);

    sprintf(path, "/dev/i2c-%d", busnum);
    dev->fd = open(path, O_RDWR);
    ASSERT(dev->fd > 0);
    dev->slave = slave & 0xffff;

    return dev;
}

static void closeDevice(struct bq_device *dev)
{
    ASSERT(dev);
    ASSERT(dev->fd > 0);
    close(dev->fd);
    free(dev);
}

static int writeDevice(struct bq_device *dev, unsigned char *data, size_t size)
{
    int ret;
    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data rw;

    msgs[0].addr = dev->slave;
    msgs[0].flags = 0;
    msgs[0].len = size;
    msgs[0].buf = data;

    rw.msgs = msgs;
    rw.nmsgs = 1;

    ret = ioctl(dev->fd, I2C_RDWR, &rw);
    if(ret < 0) {
        ERR("%s: return %d\n", __func__, ret);
        perror(__func__);
    }

    return ret;
}

static int readDevice(
        struct bq_device *dev, unsigned char reg, unsigned char *buf, int buflen)
{
    int ret;
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data rw;

    msgs[0].addr = dev->slave;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(reg);
    msgs[0].buf = &reg;
    msgs[1].addr = msgs[0].addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = buflen;
    msgs[1].buf = buf;

    rw.msgs = msgs;
    rw.nmsgs = 2;

    ret = ioctl(dev->fd, I2C_RDWR, &rw);
    if(ret < 0) {
        ERR("%s: return %d\n", __func__, ret);
        perror(__func__);
        return ret;
    }

    return ret;
}

/*****************************************************************************/

static int readSimple(struct bq_device *dev, unsigned char reg, int single)
{
    int ret;
    unsigned char data[2];

    ret = readDevice(dev, reg, data, single ? 1 : 2);
    if (ret < 0)
        return ret;

    if (!single)
        ret = (data[1] << 8) | (data[0]);
    else
        ret = data[0];

    //DBG("%s(0x%x) = 0x%x\n", __func__, reg, ret);

    return ret;
}

/*****************************************************************************/

static void writeControl(struct bq_device *dev, int subcmd)
{
    int status;
    unsigned char data[3];

    data[0] = BQ27X00_REG_CNTL;
    data[1] = subcmd & 0xff;
    data[2] = (subcmd >> 8) & 0xff;
    writeDevice(dev, data, 3);
}

static int readControl(struct bq_device *dev, int subcmd)
{
    int status;

    writeControl(dev, subcmd);
    usleep(1000);

    status = readSimple(dev, BQ27X00_REG_CNTL, 0);
    ASSERT(status >= 0);
    //DBG("CONTROL.0x%04x=0x%x\n", subcmd, status);

    return status;
}

static void bq_test_voltage(struct bq_device *dev)
{
    int flags;
    flags = readSimple(dev, BQ27X00_REG_VOLT, 0);
    DBG("BQ27X00_REG_VOLT = 0x%x\n", flags);
}

static int bq_test_flags(struct bq_device *dev)
{
    int flags;
    flags = readSimple(dev, BQ27X00_REG_FLAGS, 0);
    DBG("BQ27X00_REG_FLAGS = 0x%x\n", flags);
    return flags;
}

static void unsealDataFlash(struct bq_device *dev)
{
    unsigned char data[3];

    data[0] = BQ27X00_REG_CNTL;
    data[1] = 0x14;
    data[2] = 0x04;
    writeDevice(dev, data, 3);
    usleep(1000);

    data[0] = BQ27X00_REG_CNTL;
    data[1] = 0x72;
    data[2] = 0x36;
    writeDevice(dev, data, 3);
    usleep(1000);

    DBG("UNSEALED\n");
}

static void sealDataFlash(struct bq_device *dev)
{
    unsigned char data[3];

    data[0] = BQ27X00_REG_CNTL;
    data[1] = 0x20;
    data[2] = 0x00;
    writeDevice(dev, data, 3);
    DBG("SEALED\n");
}

/* should be called in UNSEALED state */
static void DataFlashClass(struct bq_device *dev, unsigned char classid)
{
    unsigned char data[2];
    data[0] = 0x3e;
    data[1] = classid;
    writeDevice(dev, data, 2);
}

static void DataFlashBlock(struct bq_device *dev, unsigned char blockid)
{
    unsigned char data[2];
    data[0] = 0x3f;
    data[1] = blockid;
    writeDevice(dev, data, 2);
}

static void BlockDataControl(struct bq_device *dev)
{
    unsigned char data[2];
    data[0] = 0x61;
    data[1] = 0x00;
    writeDevice(dev, data, 2);
}

#if 1
static int ReadBlockData(struct bq_device *dev, int offset, unsigned char *buf, int buflen)
{
    unsigned char reg;
    reg = 0x40 + (offset & 0x1f);
    return readDevice(dev, reg, buf, buflen);
}
#else
static int ReadBlockData(struct bq_device *dev, int offset, unsigned char *buf, int buflen)
{
    int i;
    unsigned char reg;

    for (i = 0; i < buflen; i++) {
        buf[i] = readSimple(dev, 0x40 + (offset & 0x1f) + i, 1);
    }
    return buflen;
}
#endif  /*0*/

static int WriteBlockData(struct bq_device *dev, int offset, unsigned char *buf, size_t buflen)
{
    unsigned char data[36], *p;
    data[0] = 0x40 + (offset & 0x1f);
    memcpy(&data[1], buf, buflen);
    return writeDevice(dev, data, buflen + 1);
}

#define WriteBlockDataH1                WriteBlockDataU1
#define WriteBlockDataH2                WriteBlockDataU2
#define WriteBlockDataH4                WriteBlockDataU4
#define WriteBlockDataI1                WriteBlockDataU1
#define WriteBlockDataI2                WriteBlockDataU2
#define WriteBlockDataI4                WriteBlockDataU4

static void WriteBlockDataU1(struct bq_device *dev, int offset, unsigned int value)
{
    setU1(&dev->block[offset], value & 0xff);
    WriteBlockData(dev, offset, &dev->block[offset], 1);
}

static void WriteBlockDataU2(struct bq_device *dev, int offset, unsigned int value)
{
    setU2(&dev->block[offset], value & 0xffff);
    WriteBlockData(dev, offset, &dev->block[offset], 2);
}

static void WriteBlockDataU4(struct bq_device *dev, int offset, unsigned int value)
{
    setU4(&dev->block[offset], value);
    WriteBlockData(dev, offset, &dev->block[offset], 4);
}

static int ReadBlockDataChecksum(struct bq_device *dev)
{
    int checksum;
    checksum = readSimple(dev, 0x60, 1);
    DBG("CHECKSUM = 0x%x\n", checksum);
    return checksum;
}

static void WriteBlockDataChecksum(struct bq_device *dev, unsigned char checksum)
{
    unsigned char data[2];
    data[0] = 0x60;
    data[1] = checksum;
    writeDevice(dev, data, 2);
}

static void dumpDataFlash(struct bq_device *dev, int classid, int blockid)
{
    int i;
    unsigned char sum;
    unsigned char block[32];

    usleep(1000);

    memset(block, 0, sizeof(block));

    INFO("Dump Flash (classid=%d, blockid=%d)\n", classid, blockid);

    DataFlashClass(dev, classid & 0xff);
    usleep(1000);
    DataFlashBlock(dev, blockid & 0xff);
    usleep(1000);
    /* transfer a block from data flash location to command space */
    BlockDataControl(dev);
    usleep(1000);
    ReadBlockData(dev, 0, block, sizeof(block));
    usleep(1000);

    for (i = sum = 0; i < 32; i++) {
        INFO("[%d] = 0x%x\n", i + blockid * 32, block[i]);
        sum += block[i];
    }
    DBG("my checksum = 0x%x\n", 255 - sum);
    ReadBlockDataChecksum(dev);
}

static void loadDataFlash(struct bq_device *dev, int classid, int blockid)
{
    unsigned int i;
    unsigned char checksum = 0;

    dev->classid = classid;
    dev->blockid = blockid;
    memset(dev->block, 0, sizeof(dev->block));

    /* transfer a block from data flash location to command space */
    BlockDataControl(dev);
    usleep(1000);
    DataFlashClass(dev, classid & 0xff);
    usleep(1000);
    DataFlashBlock(dev, blockid & 0xff);
    usleep(1000);

    ReadBlockData(dev, 0, dev->block, sizeof(dev->block));
    dev->checksum = ReadBlockDataChecksum(dev) & 0xff;

    for (i = 0; i < sizeof(dev->block); i++)
        checksum += dev->block[i];

    DBG("Read Checksum = 0x%x, My Checksum = 0x%x\n", dev->checksum, 255 - checksum);
}

static unsigned char calChecksum(unsigned char *buf, size_t buflen)
{
    unsigned int i;
    unsigned char sum = 0;

    for (i = 0; i < buflen; i++)
        sum += buf[i];

    return (255 - sum);
}

static void storeDataFlash(struct bq_device *dev)
{
    unsigned char checksum;
    DBG("===== %s(classid=%d, blockid=%d) =====\n", __func__, dev->classid, dev->blockid);
    checksum = calChecksum(dev->block, sizeof(dev->block));
    if (dev->checksum != checksum) {
        dev->checksum = checksum;
        WriteBlockDataChecksum(dev, dev->checksum);
        usleep(1000);
    }
}

/*****************************************************************************/

static void enterCalMode(struct bq_device *dev)
{
    int status;
    unsigned char data[3];

    do {
        data[0] = BQ27X00_REG_CNTL;
        data[1] = 0x2D;
        data[2] = 0x00;
        writeDevice(dev, data, 3);
        usleep(1000);

        data[0] = BQ27X00_REG_CNTL;
        data[1] = 0x81;
        data[2] = 0x00;
        writeDevice(dev, data, 3);
        usleep(1000);

        status = bq_test_flags(dev);
    } while (status < 0 || (!(status & 0x1000)));

    DBG("Entered to the calibration mode.\n");
}

static void exitCalMode(struct bq_device *dev)
{
    int status;
    unsigned char data[3];

    data[0] = BQ27X00_REG_CNTL;
    data[1] = 0x80;
    data[2] = 0x00;
    writeDevice(dev, data, 3);
    usleep(1000);

    DBG("Exited from the calibration mode.\n");
}

static void calibCCOffset(struct bq_device *dev)
{
    int status;
    unsigned char data[3];

    do {
        data[0] = BQ27X00_REG_CNTL;
        data[1] = 0x0A;
        data[2] = 0x00;
        writeDevice(dev, data, 3);
        usleep(1000);

        status = readControl(dev, SUBCMD_CONTROL_STATUS);
    } while (status < 0 || (!(status & 0x800)));

    DBG("Proceeding CC Offset calibration.\n");

    do {
        sleep(1);
        status = readControl(dev, SUBCMD_CONTROL_STATUS);
    } while (status < 0 || (status & 0x800));

    DBG("Done CC Offset calibration.\n");

    data[0] = BQ27X00_REG_CNTL;
    data[1] = 0x0B;
    data[2] = 0x00;
    writeDevice(dev, data, 3);
    usleep(1000);
}

/*****************************************************************************/

#define BQ27520_I2C_BUS                     (6)
#define BQ27520_I2C_SLAVE                   (0x55)

void bq_test(void)
{
    struct bq_device *dev;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    /* DEVICE_TYPE */
    readControl(dev, SUBCMD_DEVICE_TYPE);
    bq_test_voltage(dev);
    bq_test_flags(dev);

    readControl(dev, SUBCMD_CONTROL_STATUS);

    unsealDataFlash(dev);
    /* if we enable below line, the modification will fail.
    readControl(dev, SUBCMD_CONTROL_STATUS);
    */

    //bq_test_modify_flash(dev, 82, 0);
    //dumpDataFlash(dev, 2, 0);
    dumpDataFlash(dev, 2, 0);
    dumpDataFlash(dev, 48, 0);
    dumpDataFlash(dev, 82, 0);
    sealDataFlash(dev);

    closeDevice(dev);
}

void BQ_DumpFlash(int classid, int blockid)
{
    unsigned int i;
    struct bq_device *dev;
    static int classids[] = {
        2, 32, 34, 36, 48, 49, 64, 68, 57, 80, 81, 82, 83,
        84, 87, 88, 91, 92, 93, 94, 104, 106, 107, 112,
    };

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);

    if (classid > 0) {
        dumpDataFlash(dev, classid, blockid);
    }
    else {
        for (i = 0; i < ARRAY_SIZE(classids); i++) {
            dumpDataFlash(dev, classids[i], 0);
        }
    }

    sealDataFlash(dev);
    closeDevice(dev);
}

void BQ_Calibrate(void)
{
    struct bq_device *dev;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);
    enterCalMode(dev);
    calibCCOffset(dev);
    exitCalMode(dev);
    sealDataFlash(dev);

    closeDevice(dev);
}

void BQ_Status(void)
{
    int value;
    unsigned int i;
    struct bq_device *dev;
    struct bq_status_item *psi;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);
    writeControl(dev, SUBCMD_IT_ENABLE);
    usleep(1000);
    sealDataFlash(dev);
    usleep(1000);

    for (i = 0; i < ARRAY_SIZE(show_stats); i++) {
        psi = &show_stats[i];
        if (psi->is_subcmd)
            value = readControl(dev, psi->addr);
        else
            value = readSimple(dev, psi->addr, 0);

        INFO("[0x%02x] = 0x%04x ;%s\n", psi->addr, value, psi->name);
    }

#if 0
    value = readControl(dev, SUBCMD_CONTROL_STATUS);
    INFO("SUBCMD_CONTROL_STATUS=0x%x\n", value);

    value = readControl(dev, SUBCMD_IT_ENABLE);
    INFO("SUBCMD_IT_ENABLE=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_TEMP, 0);
    INFO("BQ27X00_REG_TEMP=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_VOLT, 0);
    INFO("BQ27X00_REG_VOLT=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_FLAGS, 0);
    INFO("BQ27X00_REG_FLAGS=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_RM, 0);
    INFO("BQ27X00_REG_RM=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_FCC, 0);
    INFO("BQ27X00_REG_FCC=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_NAC, 0);
    INFO("BQ27X00_REG_NAC=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_CC, 0);
    INFO("BQ27X00_REG_CC=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_AI, 0);
    INFO("BQ27X00_REG_AI=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_SOC, 0);
    INFO("BQ27X00_REG_SOC=0x%x\n", value);

    value = readSimple(dev, BQ27X00_REG_OP_CONFIG, 0);
    INFO("BQ27X00_REG_OP_CONFIG=0x%x\n", value);
#endif  /*0*/

    closeDevice(dev);
}

#define CC_THRESHOLD                (2700)
#define DESIGN_CAPACITY             (2400)
#define DESIGN_VOLTAGE              (4200)
#define WEAK_BATTER_VOLTAGE         (3400)

void BQ_FlashData(void)
{
    int value;
    struct bq_device *dev;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);

    /* classid = 48 **************/
    loadDataFlash(dev, 48, 0);
    usleep(1000);

    WriteBlockDataI2(dev, 7, CC_THRESHOLD);
    usleep(1000);
    WriteBlockDataI2(dev, 10, DESIGN_CAPACITY);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    /* classid = 82 **************/
    loadDataFlash(dev, 82, 0);
    usleep(1000);

    WriteBlockDataH1(dev, 0, 1);
    usleep(1000);
    WriteBlockDataI2(dev, 2, DESIGN_CAPACITY);
    usleep(1000);
    WriteBlockDataI2(dev, 7, DESIGN_CAPACITY);
    usleep(1000);
    WriteBlockDataI2(dev, 16, 2);
    usleep(1000);
    WriteBlockDataU2(dev, 20, 20);
    usleep(1000);
    WriteBlockDataU2(dev, 22, 1000);
    usleep(1000);
    WriteBlockDataI2(dev, 24, DESIGN_VOLTAGE);
    usleep(1000);
    WriteBlockDataI2(dev, 26, DESIGN_VOLTAGE);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    /* classid = 83 **************/
    loadDataFlash(dev, 83, 0);
    usleep(1000);

    WriteBlockDataI2(dev, 2, DESIGN_CAPACITY);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    /* classid = 84 **************/
    loadDataFlash(dev, 84, 0);
    usleep(1000);

    WriteBlockDataI2(dev, 2, DESIGN_CAPACITY);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    /* to apply data written */
    writeControl(dev, SUBCMD_RESET);
    usleep(1000);

    sealDataFlash(dev);

    closeDevice(dev);
}

void BQ_SetBatteryDetect(int force, int insert)
{
    int value;
    struct bq_device *dev;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);

    loadDataFlash(dev, 64, 0);
    usleep(1000);

    /* OpConfigB[BIE] = 0 */
    if (force)
        WriteBlockDataH1(dev, 11, dev->block[11] & ~0x40);
    else
        WriteBlockDataH1(dev, 11, dev->block[11] | 0x40);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    writeControl(dev, SUBCMD_RESET);
    usleep(1000);

    sealDataFlash(dev);
    usleep(1000);

    if (force) {
        INFO("%s Battery forcibly\n", insert ? "Insert" : "Remove");
        writeControl(dev, insert ? SUBCMD_BAT_INSERT : SUBCMD_BAT_REMOVE);
        usleep(1000);
    }

    value = readSimple(dev, BQ27X00_REG_FLAGS, 0);
    INFO("BQ27X00_REG_FLAGS=0x%x\n", value);

    closeDevice(dev);
}

void BQ_SetWriteTemp(int wrtemp, int tempC)
{
    int value;
    struct bq_device *dev;

    dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(dev);

    unsealDataFlash(dev);
    usleep(1000);

    loadDataFlash(dev, 64, 0);
    usleep(1000);

    /* OpConfigB[WRTEMP] = 1 */
    if (wrtemp)
        WriteBlockDataH1(dev, 11, dev->block[11] | 0x80);
    else
        WriteBlockDataH1(dev, 11, dev->block[11] & ~0x80);
    usleep(1000);

    storeDataFlash(dev);
    usleep(1000);

    writeControl(dev, SUBCMD_RESET);
    usleep(1000);

    sealDataFlash(dev);
    usleep(1000);

    if (wrtemp) {
        int tempK;
        unsigned char data[3];
        tempK = (tempC + 273) * 10;
        INFO("Write Temperature = %d(K*10=0x%x)\n", tempC, tempK);
        data[0] = BQ27X00_REG_TEMP;
        data[1] = tempK & 0xff;
        data[2] = (tempK >> 8) & 0xff;
        writeDevice(dev, data, 3);
        usleep(1000);
    }

    value = readSimple(dev, BQ27X00_REG_TEMP, 0);
    INFO("BQ27X00_REG_TEMP=0x%x\n", value);

    closeDevice(dev);
}

/*****************************************************************************/

enum {
    CMD_OPENFLASH = 1,
    CMD_CLOSEFLASH,
    CMD_WRITE,
    CMD_READ,
    CMD_PRINT,
    CMD_USLEEP,
    CMD_RESET,
    CMD_SET,
};

struct BQScriptEntry;
struct BQScriptEngine;

struct BQScriptCommand {
    const char *name;
    int id;
    int (*run)(struct BQScriptEngine *engine, const struct BQScriptEntry *entry);
};

#define MAX_ARGC                    (32)

struct BQScriptEntry {
    const struct BQScriptCommand *cmd;
    unsigned argc;
    char *argv[MAX_ARGC];
};

struct BQScriptEngine {
    struct bq_device *dev;
    FILE *file;
    char linebuf[512];
};

/*****************************************************************************/

static int cmd_openFlash(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    int classid, blockid;

    ASSERT(engine);
    ASSERT(entry);

    if (entry->argc != 2) {
        return -1;
    }

    ASSERT(entry->argv[0]);
    classid = strtol(entry->argv[0], NULL, 0);
    ASSERT(entry->argv[1]);
    blockid = strtol(entry->argv[1], NULL, 0);

    INFO("%s(classid=%d, blockid=%d)\n", entry->cmd->name, classid, blockid);
    loadDataFlash(engine->dev, classid, blockid);

    return 0;
}

static int cmd_closeFlash(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    ASSERT(engine);
    ASSERT(entry);
    INFO("%s\n", entry->cmd->name);
    storeDataFlash(engine->dev);
    return 0;
}

static int cmd_readFlash(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    ASSERT(engine);
    ASSERT(entry);
    INFO("%s: Not supported.\n", entry->cmd->name);
    return -1;
}

#define DATA_TYPE_I             (0 << 4)
#define DATA_TYPE_H             (1 << 4)
#define DATA_TYPE_U             (2 << 4)
#define DATA_SIZE_1             (1)
#define DATA_SIZE_2             (2)
#define DATA_SIZE_4             (4)

static int getDataType(const char *typestr)
{
    int type = 0;

    if (strlen(typestr) != 2) {
        DBG("%s:length=%d\n", typestr, strlen(typestr));
        return -1;
    }

    switch(typestr[0]) {
        case 'i':
        case 'I':
            type |= DATA_TYPE_I;
            break;
        case 'h':
        case 'H':
            type |= DATA_TYPE_H;
            break;
	case 'u':
	case 'U':
	    type |= DATA_TYPE_U;
            break;

        default:
            return -1;
    }

    switch(typestr[1]) {
        case '1':
            type |= DATA_SIZE_1;
            break;
        case '2':
            type |= DATA_SIZE_2;
            break;
        case '4':
            type |= DATA_SIZE_4;
            break;
        default:
            return -1;
    }

    return type;
}

static int cmd_writeFlash(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    int type;
    int offset;
    unsigned value;

    ASSERT(engine);
    ASSERT(entry);

    if (entry->argc != 3) {
        DBG("Invalid parameters. We need <type> <offset> <data>\n");
        return -1;
    }

    ASSERT(entry->argv[0]);
    type = getDataType(entry->argv[0]);
    ASSERT(type >= 0);

    ASSERT(entry->argv[1]);
    offset = strtol(entry->argv[1], NULL, 0);

    ASSERT(entry->argv[2]);
    value = strtoul(entry->argv[2], NULL, 0);

    INFO("%s(%d, %d)\n", entry->cmd->name, offset, value);
    switch (type) {
        case    DATA_TYPE_H | DATA_SIZE_1:
            WriteBlockDataH1(engine->dev, offset, value & 0xff);
            break;
        case    DATA_TYPE_H | DATA_SIZE_2:
            WriteBlockDataH2(engine->dev, offset, value & 0xffff);
            break;
        case    DATA_TYPE_H | DATA_SIZE_4:
            WriteBlockDataH4(engine->dev, offset, value);
            break;
        case    DATA_TYPE_I | DATA_SIZE_1:
            WriteBlockDataI1(engine->dev, offset, value & 0xff);
            break;
        case    DATA_TYPE_I | DATA_SIZE_2:
            WriteBlockDataI2(engine->dev, offset, value & 0xffff);
            break;
        case    DATA_TYPE_I | DATA_SIZE_4:
            WriteBlockDataI4(engine->dev, offset, value);
            break;
        case    DATA_TYPE_U | DATA_SIZE_1:
            WriteBlockDataU1(engine->dev, offset, value & 0xff);
            break;
        case    DATA_TYPE_U | DATA_SIZE_2:
            WriteBlockDataU2(engine->dev, offset, value & 0xffff);
            break;
        case    DATA_TYPE_U | DATA_SIZE_4:
            WriteBlockDataU4(engine->dev, offset, value);
            break;
    }

    return 0;
}

static int cmd_usleep(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    int msec;

    ASSERT(engine);
    ASSERT(entry);
    ASSERT(entry->argv[0]);
    msec = strtol(entry->argv[0], NULL, 0);
    INFO("%s(%d)\n", entry->cmd->name, msec);
    usleep(msec);

    return 0;
}

static int cmd_print(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    ASSERT(engine);
    ASSERT(entry);
    DBG("%s: Not supported.\n", entry->cmd->name);
    return -1;
}

static int cmd_reset(
        struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    ASSERT(engine);
    ASSERT(entry);
    INFO("%s\n", entry->cmd->name);
    writeControl(engine->dev, SUBCMD_RESET);
    return 0;
}

/*****************************************************************************/

static struct BQScriptCommand BQCmdTable[] = {
    { "OF", CMD_OPENFLASH, cmd_openFlash },
    { "OPENFLASH", CMD_OPENFLASH, cmd_openFlash },
    { "CF", CMD_CLOSEFLASH, cmd_closeFlash },
    { "CLOSEFLASH", CMD_CLOSEFLASH, cmd_closeFlash },
    { "WR", CMD_WRITE, cmd_writeFlash },
    { "WRITE", CMD_WRITE, cmd_writeFlash },
    { "RD", CMD_READ, cmd_readFlash },
    { "READ", CMD_READ, cmd_readFlash },
    { "US", CMD_USLEEP, cmd_usleep },
    { "USLEEP", CMD_USLEEP, cmd_usleep },
    { "PR", CMD_PRINT, cmd_print },
    { "PRINT", CMD_PRINT, cmd_print },
    { "RESET", CMD_RESET, cmd_reset },
    { "RST", CMD_RESET, cmd_reset },
};

/*****************************************************************************/

static const struct BQScriptCommand *lookupCommand(const char *cmd)
{
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(BQCmdTable); i++) {
        if (strcasecmp(cmd, BQCmdTable[i].name))
            continue;

        return &BQCmdTable[i];
    }

    return (const struct BQScriptCommand *)NULL;
}

static int getNextEntry(struct BQScriptEngine *engine, struct BQScriptEntry *entry)
{
    int ret = -1;
    char *s, *e, *token;
    const char *delim = " \t";

    entry->argc = 0;
    for ( ; ; ) {
        if (feof(engine->file)) {
            ret = 0;
            break;
        }

        s = fgets(engine->linebuf, sizeof(engine->linebuf) - 1, engine->file);
        if (!s) {
            continue;
        }

        /* command name */
        token = strtok(s, delim);
        if (token[0] == '#') {
            DBG("COMMENT=\"%s\"\n", token);
            continue;
        }

        /* skip new-line */
        s = strrchr(token, '\n');
        if (s)
            *s = 0;

        if (strlen(token) == 0)
            continue;

        DBG("token=[%s]\n", token);

        entry->cmd = lookupCommand(token);
        if (!entry->cmd) {
            DBG("%s: Invalid Command\n", token);
            ret = -1;
            break;
        }

        DBG("%s: Found command(id=%d)\n", token, entry->cmd->id);

        ret = 1;
        for ( ; ; ) {
            token = strtok(NULL, delim);
            if (!token)
                break;

            entry->argv[entry->argc] = strdup(token);
            ASSERT(entry->argv[entry->argc]);
            entry->argc++;
        }

        break;
    }

    return ret;
}

static void releaseEntry(struct BQScriptEntry *entry)
{
    unsigned i;

    for (i = 0; i < entry->argc; i++) {
        ASSERT(entry->argv[i]);
        free(entry->argv[i]);
        entry->argv[i] = NULL;
    }
}

static int runScript(struct BQScriptEngine *engine, const struct BQScriptEntry *entry)
{
    int ret;
    ASSERT(entry->cmd->run);
    ret = entry->cmd->run(engine, entry);
    DBG("%s: Run result: ret=%d\n", entry->cmd->name, ret);
    return ret;
}

static struct BQScriptEngine *openScriptEngine(const char *path)
{
    struct BQScriptEngine *engine;

    engine = calloc(1, sizeof(*engine));
    ASSERT(engine && "No memory");

    engine->file = fopen(path, "r");
    ASSERT(engine->file && "Unknown filename");

    engine->dev = openDevice(BQ27520_I2C_BUS, BQ27520_I2C_SLAVE);
    ASSERT(engine->dev);

    unsealDataFlash(engine->dev);

    return engine;
}

static void closeScriptEngine(struct BQScriptEngine *engine)
{
    ASSERT(engine);
    ASSERT(engine->file);
    ASSERT(engine->dev);

    sealDataFlash(engine->dev);
    closeDevice(engine->dev);

    fclose(engine->file);
    free(engine);
}

void BQ_RunFlashScript(const char *path)
{
    int ret;
    struct BQScriptEntry entry;
    struct BQScriptEngine *engine;

    engine = openScriptEngine(path);
    ASSERT(engine);

    for ( ; ; ) {

        ret = getNextEntry(engine, &entry);

        if (ret < 0) {
            DBG("Parse error(ret=%d)\n", ret);
            break;
        }
        else if (ret == 0) {
            DBG("EOF\n");
            break;
        }

        ret = runScript(engine, &entry);
        releaseEntry(&entry);

        if (ret < 0) {
            DBG("runScript: Error occurred.\n");
            break;
        }
    }

    closeScriptEngine(engine);
}
