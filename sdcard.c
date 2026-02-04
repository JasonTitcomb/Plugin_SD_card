/*
  sdcard.c - streaming plugin for SDCard/FatFs

  Part of grblHAL

  Copyright (c) 2018-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "sdcard.h"

#if FS_ENABLE & FS_SDCARD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFLEN 80

#if defined(ESP_PLATFORM) || defined(STM32_PLATFORM) || defined(__LPC17XX__) || defined(__IMXRT1062__) || defined(__MSP432E401Y__)
#define NEW_FATFS
#endif


#include "grbl/report.h"
// #include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/stream_file.h"
#include "grbl/vfs.h"
#include "grbl/task.h"
#include "grbl/system.h" // for sys.abort

#include "fs_fatfs.h"

#if defined(NEW_FATFS)
static char dev[10] = "";
#endif

// https://e2e.ti.com/support/tools/ccs/f/81/t/428524?Linking-error-unresolved-symbols-rom-h-pinout-c-

/* uses fatfs - http://www.elm-chan.org/fsw/ff/00index_e.html */

static FATFS *fatfs = NULL;
static bool mount_changed = false, realtime_report_subscribed = false, sd_detectable = false;
static xbar_t *detect_pin = NULL;
static sdcard_events_t sdcard;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;
static driver_setup_ptr driver_setup;
static settings_changed_ptr settings_changed;

// forward declarations
static void onRealtimeReport(stream_write_ptr stream_write, report_tracking_flags_t report);
static int32_t file_upload_read(void);

typedef struct
{
    vfs_file_t *file;       // target SD file
    const char *filename;   // for reporting
    uint32_t expected_size; // total bytes expected
    uint32_t received;      // bytes received so far
    uint32_t crc;           // optional CRC32
    bool active;
} upload_t;

static upload_t upload;

// backup of original stream reader
static stream_read_ptr stream_read_backup;
// --- optional CRC32 helper ---
static uint32_t crc32_update(uint32_t crc, uint8_t data);

#ifdef __MSP432E401Y__
/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD fatfs_getFatTime(void)
{
    return ((2007UL - 1980) << 25) // Year = 2007
           | (6UL << 21)           // Month = June
           | (5UL << 16)           // Day = 5
           | (11U << 11)           // Hour = 11
           | (38U << 5)            // Min = 38
           | (0U >> 1)             // Sec = 0
        ;
}
#endif

static uint32_t crc32_update(uint32_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++)
        crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    return crc;
}

static on_realtime_report_ptr prev_report = NULL;
static void onUploadRealtimeReport(stream_write_ptr sw, report_tracking_flags_t report)
{
    if (prev_report)
        prev_report(sw, report);

    if (!report.all || !upload.active)
        return;

    float percent = 0.0f;
    if (upload.expected_size)
        percent = 100.0f * upload.received / upload.expected_size;

    char pct[8];
    strcpy(pct, ftoa(percent, 1));

    char crcmsg[16];
    snprintf(crcmsg, sizeof(crcmsg), "%08lX", upload.crc);

    sw("|UP:");
    sw(pct);
    sw(",CRC32:");
    sw(crcmsg);
    sw(",FILE:");
    sw(upload.filename);
}


// --- called by $FUP command to start upload ---
status_code_t file_upload(const char *fname, uint32_t size)
{
    if (upload.active)
        return Status_InvalidStatement; // already uploading

    upload.file = vfs_open(fname, "w");
    if (!upload.file)
        return Status_FileOpenFailed;

    upload.filename = fname;
    upload.expected_size = size;
    upload.received = 0;
    upload.crc = 0;
    upload.active = true;

    // override stream reader
    stream_read_backup = hal.stream.read;
    hal.stream.read = file_upload_read;

    // register status hook
    grbl.on_realtime_report = onUploadRealtimeReport;
    return Status_OK;
}

// --- called by hal.stream.read during upload ---
static int32_t file_upload_read(void)
{

    if (!upload.active)
        return -1;

    // Abort upload if system abort or cancel is set
    if (sys.abort || sys.cancel) {
        if (upload.file)
            vfs_close(upload.file);
            
        upload.active = false;
        hal.stream.read = stream_read_backup;
        report_message("Upload aborted by user", Message_Warning);
        return -1;
    }

    int16_t c = stream_read_backup();
    if (c < 0)
        return -1; // nothing available

    uint8_t b = (uint8_t)c;

    // write to SD via VFS
    size_t written = vfs_write(&b, 1, 1, upload.file);
    if (written != 1)
    {
        // handle SD write error: abort upload
        vfs_close(upload.file);
        upload.active = false;
        hal.stream.read = stream_read_backup;
        report_message("Upload failed: SD write error", Message_Error);
        return -1;
    }

    upload.received++;

    upload.crc = crc32_update(upload.crc, b);


    // finish condition
    if (upload.received >= upload.expected_size)
    {
        vfs_close(upload.file);
        upload.active = false;
        hal.stream.read = stream_read_backup;
         char uploadmsg[64];
        snprintf(uploadmsg, sizeof(uploadmsg), "File: %s, Bytes: %lu, CRC32: %08lX", upload.filename, upload.received, upload.crc);
        report_message(uploadmsg, Message_Info);    
        report_message("Upload complete", Message_Info);
    }

    return -1; // do not feed parser
}

// expects command arguments: "<filename> <size>"
static status_code_t sd_command_upload_start(sys_state_t state, char *args)
{
    // parse the file name and size from args
    char *filename = NULL;
    char *size = NULL;

    // Skip the '=' if present
    char *start = args[0] == '=' ? args + 1 : args;

    // Split by comma
    filename = strtok(start, ",");
    size = strtok(NULL, ",");

    // filename now points to "job.nc"
    // size now points to "123456"

    // Check for exactly two arguments
    if (filename && size && strtok(NULL, ",") == NULL)
    {
        // Valid: two arguments
        return file_upload(filename, atoi(size));
    }
    else
    {
        // Invalid: wrong number of arguments
        return Status_InvalidStatement;
    }
}

static bool sdcard_mount(void)
{
    static FATFS *fs = NULL;

    bool is_mounted = !!fatfs;

    if (sdcard.on_mount)
    {

        char *mdev = sdcard.on_mount(&fatfs);

        if (fatfs != NULL)
        {
#ifdef NEW_FATFS
            if (mdev)
                strcpy(dev, mdev);
#endif
        }
    }
    else
    {

        if (fs == NULL)
            fs = malloc(sizeof(FATFS));

#ifdef NEW_FATFS
        if (fs && (f_mount(fs, dev, 1) == FR_OK))
#else
        if (fs && (f_mount(0, fs) == FR_OK))
#endif
            fatfs = fs;
        else
            fatfs = NULL;
    }

    if ((mount_changed = is_mounted != !!fatfs) && !realtime_report_subscribed)
    {
        realtime_report_subscribed = true;
        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport; // Add mount status changes and job percent complete to real time report
    }

    if (fatfs != NULL)
        fs_fatfs_mount("/");

    return fatfs != NULL;
}

static void sdcard_auto_mount(void *data)
{
    if (fatfs == NULL && !sdcard_mount())
        report_message("SD card automount failed", Message_Info);
}

static bool sdcard_unmount(void)
{
    if (fatfs)
    {
        if (sdcard.on_unmount)
            mount_changed = sdcard.on_unmount(&fatfs);
#ifdef NEW_FATFS
        else
            mount_changed = f_unmount(dev) == FR_OK;
#else
        else
            mount_changed = f_mount(0, NULL) == FR_OK;
#endif
        if (mount_changed && fatfs)
        {
            fatfs = NULL;
            vfs_unmount("/");
        }
    }

    return fatfs == NULL;
}

static status_code_t sd_cmd_mount(sys_state_t state, char *args)
{
    return sdcard_mount() ? Status_OK : Status_SDMountError;
}

static status_code_t sd_cmd_unmount(sys_state_t state, char *args)
{
    return fatfs ? (sdcard_unmount() ? Status_OK : Status_SDMountError) : Status_SDNotMounted;
}

#if FF_FS_READONLY == 0 && FF_USE_MKFS == 1

static status_code_t sd_cmd_format(sys_state_t state, char *args)
{
    status_code_t status = Status_InvalidStatement;

    if (fatfs)
    {

        vfs_drive_t *drive = vfs_get_drive("/");

        if (drive->fs && !strcmp(args, "yes"))
        {

            report_message("Formatting SD card...", Message_Info);

            if (vfs_drive_format(drive) == 0)
                status = !sdcard_mount() ? Status_SDMountError : Status_OK;
            else
                status = Status_FsFormatFailed;

            report_message("", Message_Plain);
        }
    }
    else
        status = Status_SDNotMounted;

    return status;
}

#endif

static void sd_detect(void *mount)
{
    if ((uint32_t)mount == 0)
        sdcard_unmount();
    else if (fatfs == NULL)
        sdcard_mount();
}

ISR_CODE void ISR_FUNC(sdcard_detect)(bool mount)
{
    task_add_immediate(sd_detect, (void *)mount);
}

static void onRealtimeReport(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if (report.all || mount_changed)
    {
        stream_write("|SD:");
        stream_write(uitoa((sd_detectable ? 2 : 0) + !!fatfs));
        mount_changed = false;
    }

    if (on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void sd_detect_pin(xbar_t *pin, void *data)
{
    if (pin->id == Input_SdCardDetect)
    {
        sd_detectable = true;
        if (pin->get_value)
            detect_pin = pin;
    }
}

static void onSettingsChanged(settings_t *settings, settings_changed_flags_t changed)
{
    static bool mount_attempted = false; // in case some other code hooked into hal.settings_changed

    settings_changed(settings, changed);

    if (!mount_attempted)
    {
        mount_attempted = true;
        sdcard_mount();
    }
}

static bool onDriverSetup(settings_t *settings)
{
    bool ok;

    settings_changed = hal.settings_changed;
    hal.settings_changed = onSettingsChanged;

    ok = driver_setup(settings);

    if (hal.settings_changed == onSettingsChanged)
        hal.settings_changed = settings_changed;

    return ok;
}

// Attempt early mount before other clients access a shared SPI bus.
void sdcard_early_mount(void)
{
    if (detect_pin == NULL || detect_pin->get_value(detect_pin) == 0.0f)
    {
        driver_setup = hal.driver_setup;
        hal.driver_setup = onDriverSetup;
    }
}

static void onReportOptions(bool newopt)
{
    on_report_options(newopt);

    if (newopt)
        hal.stream.write(",SD");
    else
        report_plugin("SDCARD", "1.27");
}

sdcard_events_t *sdcard_init(void)
{
    PROGMEM static const sys_command_t sdcard_command_list[] = {
        {"FM", sd_cmd_mount, {.noargs = On}, {.str = "mount SD card"}},
        {"FU", sd_cmd_unmount, {.noargs = On}, {.str = "unmount SD card"}},
        {"FUP", sd_command_upload_start, {}, {.str = "upload to SD card"}},
#if FF_FS_READONLY == 0 && FF_USE_MKFS == 1
        {"FF", sd_cmd_format, {}, {.str = "$FF=yes - format SD card"}},
#endif
    };

    static sys_commands_t sdcard_commands = {
        .n_commands = sizeof(sdcard_command_list) / sizeof(sys_command_t),
        .commands = sdcard_command_list};

    PROGMEM static const status_detail_t status_detail[] = {
        {Status_SDMountError, "SD Card mount failed."},
        {Status_SDNotMounted, "SD Card not mounted."}};

    static error_details_t error_details = {
        .errors = status_detail,
        .n_errors = sizeof(status_detail) / sizeof(status_detail_t)};

    hal.driver_cap.sd_card = On;

    hal.enumerate_pins(false, sd_detect_pin, NULL);

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    errors_register(&error_details);
    system_register_commands(&sdcard_commands);

    if (settings.fs_options.sd_mount_on_boot)
        task_run_on_startup(sdcard_auto_mount, NULL);

    return &sdcard;
}

FATFS *sdcard_getfs(void)
{
    if (fatfs == NULL)
        sdcard_mount();

    return fatfs;
}

#endif // FS_ENABLE & FS_SDCARD
