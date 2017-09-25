#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/*****************************************************************************/

#define FLAG_DUMP                       (0x001)
#define FLAG_FLASH_DATA                 (0x002)
#define FLAG_CALIBRATE                  (0x004)
#define FLAG_STATUS                     (0x008)
#define FLAG_INSERT_BATTERY             (0x010)
#define FLAG_REMOVE_BATTERY             (0x020)
#define FLAG_WRITE_TEMP                 (0x040)
#define FLAG_WRITE_TEMP_CLEAR           (0x080)
#define FLAG_ENABLE_BIE                 (0x100)
#define FLAG_FLASH_SCRIPT               (0x200)

/*****************************************************************************/

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "  -i   Insert Battery\n"
            "  -I   Enable BIE\n"
            "  -r   Remove Battery\n"
            "  -t   Write Temperature 30C\n"
            "  -T   Write Temperature Clear\n"
            "  -d   Dump flash data. Needs classid\n"
            "  -b   Specify blockid for dump. Needs blockid\n"
            "  -c   Calibrate\n"
            "  -s   Status\n"
            "  -f   Run flashing script. Needs filename\n"
            "  -F   Flash initial data using internal data\n",
            program);
}

extern void BQ_Status(void);
extern void BQ_DumpFlash(int classid, int blockid);
extern void BQ_Calibrate(void);
extern void BQ_FlashData(void);
extern void BQ_SetBatteryDetect(int force, int insert);
extern void BQ_SetWriteTemp(int wrtemp, int tempC);
extern void BQ_RunFlashScript(const char *path);

int main(int argc, char **argv)
{
    int opt;
    int dumpClassid = 0;
    int dumpBlockid = 0;
    long flags = 0;
    char *filename = NULL;

    while ((opt = getopt(argc, argv, "sd:b:cf:FiIrtT")) != -1) {
        switch (opt) {
            case 'i':
                flags |= FLAG_INSERT_BATTERY;
                break;
            case 'I':
                flags |= FLAG_ENABLE_BIE;
                break;
            case 'r':
                flags |= FLAG_REMOVE_BATTERY;
                break;
            case 't':
                flags |= FLAG_WRITE_TEMP;
                break;
            case 'T':
                flags |= FLAG_WRITE_TEMP_CLEAR;
                break;
            case 's':
                flags |= FLAG_STATUS;
                break;
            case 'd':
                flags |= FLAG_DUMP;
                dumpClassid = atoi(optarg);
                break;
            case 'b':
                dumpBlockid = atoi(optarg);
                break;
            case 'f':
                flags |= FLAG_FLASH_SCRIPT;
                filename = optarg;
                break;
            case 'F':
                flags |= FLAG_FLASH_DATA;
                break;
            case 'c':
                flags |= FLAG_CALIBRATE;
                break;
            default: /* '?' */
                usage(argv[0]);
                exit(EXIT_FAILURE);
                break;
        }
    }

    if (!flags) {
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    if ((flags & (FLAG_INSERT_BATTERY | FLAG_REMOVE_BATTERY)) == 
            (FLAG_INSERT_BATTERY | FLAG_REMOVE_BATTERY)) {
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }
    else if (flags & FLAG_ENABLE_BIE)
        BQ_SetBatteryDetect(0, 0);

    if (flags & FLAG_INSERT_BATTERY)
        BQ_SetBatteryDetect(1, 1);
    else if (flags & FLAG_REMOVE_BATTERY)
        BQ_SetBatteryDetect(1, 0);

    if (flags & FLAG_WRITE_TEMP)
        BQ_SetWriteTemp(1, 35);
    else if (flags & FLAG_WRITE_TEMP_CLEAR)
        BQ_SetWriteTemp(0, 0);

    if (flags & FLAG_STATUS)
        BQ_Status();

    if (flags & FLAG_FLASH_DATA)
        BQ_FlashData();

    if (flags & FLAG_CALIBRATE)
        BQ_Calibrate();

    if (flags & FLAG_DUMP)
        BQ_DumpFlash(dumpClassid, dumpBlockid);

    if (flags & FLAG_FLASH_SCRIPT)
        BQ_RunFlashScript(filename);

    return 0;
}
