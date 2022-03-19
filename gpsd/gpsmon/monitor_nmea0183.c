/*
 * monitor_nmea0183.c - gpsmon support for NMEA devices.
 *
 * To do: Support for GPGLL, GPGBS, GPZDA, PASHR NMEA sentences.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>                   // for labs()
#include <string.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/gpsmon.h"
#include "../include/gpsdclient.h"
#include "../include/strfuncs.h"

extern const struct gps_type_t driver_nmea0183;

static timespec_t last_tick, tick_interval;
// string to store the message types seen
// 132 for no good reason, longer than the field it goes into
static char sentences[132];

/*****************************************************************************
 *
 * NMEA0183 support
 *
 *****************************************************************************/

#define SENTENCELINE    1       /* index of sentences line in the NMEA window */

/* define all window width constants at one location */
/* WIDTH shall be >= 80 */
#define WIDTH_L 25
#define WIDTH_M 27
#define WIDTH_R 30
#define WIDTH (WIDTH_L + WIDTH_M + WIDTH_R - 2)

#define HEIGHT_1 3
#define HEIGHT_2 3
#define HEIGHT_3 9
/* set to 6 for 80x24 screen, set to 7 for 80x25 screen */
#define HEIGHT_4 6
#define HEIGHT (HEIGHT_1 + HEIGHT_2 + HEIGHT_3 + HEIGHT_4)
/* max satellites we can display */
#define MAXSATS (HEIGHT_3 + HEIGHT_4 - 3)

static bool nmea_initialize(void)
{

    (void)clock_gettime(CLOCK_REALTIME, &last_tick);

    sentences[0] = '\0';

    return true;
}

static void cooked_pvt(void)
{
    char scr[128];

    if (0 < session.gpsdata.fix.time.tv_sec) {
        (void)timespec_to_iso8601(session.gpsdata.fix.time, scr, sizeof(scr));
    } else
        (void)snprintf(scr, sizeof(scr), "n/a");


    if (session.gpsdata.fix.mode >= MODE_2D) {
        deg_to_str2(deg_ddmm, session.gpsdata.fix.latitude,
                    scr, sizeof(scr), " N", " S");
    } else
        (void)strlcpy(scr, "n/a", sizeof(scr));

    if (session.gpsdata.fix.mode >= MODE_2D) {
        deg_to_str2(deg_ddmm, session.gpsdata.fix.longitude,
                    scr, sizeof(scr), " E", " W");
    } else
        (void)strlcpy(scr, "n/a", sizeof(scr));
}



/* sort the skyviews
 * Used = Y first, then used = N
 * then sort by PRN
 */
static int sat_cmp(const void *p1, const void *p2)
{
   int ret = ((struct satellite_t*)p2)->used - ((struct satellite_t*)p1)->used;
   if (ret) {
        return ret;
   }
   return ((struct satellite_t*)p1)->PRN - ((struct satellite_t*)p2)->PRN;
}

static void nmea_update(void)
{
    char **fields;

    /* can be NULL if packet was overlong */
    fields = session.nmea.field;

    if (session.lexer.outbuffer[0] == (unsigned char)'$'
                && fields != NULL && fields[0] != NULL) 
    {
        int ymax, xmax;
        timespec_t now;
        timespec_t ts_diff;

        assert(ymax > 0);
        if (strstr(sentences, fields[0]) == NULL) 
        {
            char *s_end = sentences + strlen(sentences);
            if ((int)(strlen(sentences) + strlen(fields[0])) < xmax - 2) 
            {
                *s_end++ = ' ';
                (void)strlcpy(s_end, fields[0], sizeof(sentences));
            } 
            else 
            {
                *--s_end = '.';
                *--s_end = '.';
                *--s_end = '.';
            }
        }

        /*
         * If the interval between this and last update is
         * the longest we've seen yet, boldify the corresponding
         * tag.
         */
        (void)clock_gettime(CLOCK_REALTIME, &now);
        TS_SUB(&ts_diff, &now, &last_tick);
        if (TS_GZ(&ts_diff) && TS_GT(&ts_diff, &tick_interval)) 
        {
            char *findme = strstr(sentences, fields[0]);

            tick_interval = ts_diff;
        }
        last_tick = now;

        // this is a fake, GSV not decoded here, using sats from JSON
        // fields[1] is current GSV sentence, fields[2] is total sentences
        if (4 < strlen(fields[0]) &&
            0 == strcmp(fields[0] + 2, "GSV") &&
            0 == strcmp(fields[1], fields[2])) {
            int i;
            int nsats =
                (session.gpsdata.satellites_visible <
                 MAXSATS) ? session.gpsdata.satellites_visible : MAXSATS;
            // sort, so at least we see used.
            qsort(session.gpsdata.skyview, session.gpsdata.satellites_visible,
                  sizeof( struct satellite_t), sat_cmp);
            for (i = 0; i < nsats; i++) {
                // FIXME: gnssid and svid to string s/b common to all monitors.
                char *gnssid = "  ";
                char sigid = ' ';
                switch (session.gpsdata.skyview[i].gnssid) {
                default:
                    gnssid = "  ";
                    break;
                case GNSSID_GPS:
                    gnssid = "GP";  /* GPS */
                    break;
                case GNSSID_SBAS:
                    gnssid = "SB";  /* SBAS */
                    break;
                case GNSSID_GAL:
                    gnssid = "GA";  /* GALILEO */
                    break;
                case GNSSID_BD:
                    gnssid = "BD";  /* BeiDou */
                    break;
                case GNSSID_IMES:
                    gnssid = "IM";  /* IMES */
                    break;
                case GNSSID_QZSS:
                    gnssid = "QZ";  /* QZSS */
                    break;
                case GNSSID_GLO:
                    gnssid = "GL";  /* GLONASS */
                    break;
                case GNSSID_IRNSS:
                    gnssid = "IR";  // IRNSS
                    break;
                }
                if (1 < session.gpsdata.skyview[i].sigid &&
                    8 > session.gpsdata.skyview[i].sigid) 
                {
                    /* Do not display L1, or missing */
                    /* max is 8 */
                    sigid = '0' + session.gpsdata.skyview[i].sigid;
                }
                (void)printf("%.2s%3d%c %3d %3d %2d %2.0f %c%c",
                              gnssid,
                              // svid can be 3 digits
                              session.gpsdata.skyview[i].svid,
                              sigid,
                              session.gpsdata.skyview[i].PRN,
                              // degrees, 000..359
                              (int)session.gpsdata.skyview[i].azimuth,
                              // degrees, 00..90
                              (int)session.gpsdata.skyview[i].elevation,
                              // 00-99 dB-Hz, NAN, or zero, when not tracking
                              // FIXME: check isfinite()
                              session.gpsdata.skyview[i].ss,
                              SAT_HEALTH_BAD ==
                                  session.gpsdata.skyview[i].health ? 'u' : ' ',
                              session.gpsdata.skyview[i].used ? 'Y' : 'N'
                             );
            }
            // clear the rest of the sat lines
            // use i from above loop
        }
        if (4 < strlen(fields[0]) && 0 == strcmp(fields[0] + 2, "RMC")) {
            /* time, lat, lon, course, speed */
            cooked_pvt();       /* cooked version of TPV */
        }

        if (4 < strlen(fields[0]) && 0 == strcmp(fields[0] + 2, "GSA")) {
        }


        if (4 < strlen(fields[0]) && 0 == strcmp(fields[0] + 2, "GGA")) {
        }
        if (4 < strlen(fields[0]) && 0 == strcmp(fields[0] + 2, "GST")) {
        }
    }

}

#undef SENTENCELINE

static void nmea_wrap(void)
{
}

const struct monitor_object_t nmea_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT,.min_x = WIDTH,
    .driver = &driver_nmea0183,
};

/*****************************************************************************
 *
 * Extended NMEA support
 *
 *****************************************************************************/

static void monitor_nmea_send(const char *fmt, ...)
{
    char buf[BUFSIZ];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
    va_end(ap);
    (void)monitor_control_send((unsigned char *)buf, strlen(buf));
}

/*
 * Yes, it's OK for most of these to be clones of the generic NMEA monitor
 * object except for the pointer to the GPSD driver.  That pointer makes
 * a difference, as it will automatically enable stuff like speed-switcher
 * and mode-switcher commands.  It's really only necessary to write a
 * separate monitor object if you want to change the device-window
 * display or implement device-specific commands.
 */

#if defined(GARMIN_ENABLE)
extern const struct gps_type_t driver_garmin;

const struct monitor_object_t garmin_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_garmin,
};
#endif  // GARMIN_ENABLE

extern const struct gps_type_t driver_ashtech;

#define ASHTECH_SPEED_9600 5
#define ASHTECH_SPEED_57600 8

static int ashtech_command(char line[])
{
    switch (line[0]) {
    case 'N':                   /* normal = 9600, GGA+GSA+GSV+RMC+ZDA */
        monitor_nmea_send("$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
        monitor_nmea_send("$PASHS,NME,ALL,B,OFF");
        monitor_nmea_send("$PASHS,NME,GGA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSV,A,ON");
        monitor_nmea_send("$PASHS,NME,RMC,A,ON");
        monitor_nmea_send("$PASHS,NME,ZDA,A,ON");

        monitor_nmea_send("$PASHS,INI,%d,%d,,,0,",
                          ASHTECH_SPEED_9600, ASHTECH_SPEED_9600);
        (void)sleep(6);         // it takes 4-6 sec for the receiver to reboot
        monitor_nmea_send("$PASHS,WAS,ON");     /* enable WAAS */
        break;

    case 'R':                 /* raw = 57600, normal+XPG+POS+SAT+MCA+PBN+SNV */
        monitor_nmea_send("$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
        monitor_nmea_send("$PASHS,NME,ALL,B,OFF");
        monitor_nmea_send("$PASHS,NME,GGA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSV,A,ON");
        monitor_nmea_send("$PASHS,NME,RMC,A,ON");
        monitor_nmea_send("$PASHS,NME,ZDA,A,ON");

        monitor_nmea_send("$PASHS,INI,%d,%d,,,0,",
                          ASHTECH_SPEED_57600, ASHTECH_SPEED_9600);
        (void)sleep(6);         // it takes 4-6 sec for the receiver to reboot
        monitor_nmea_send("$PASHS,WAS,ON");     /* enable WAAS */

        monitor_nmea_send("$PASHS,NME,POS,A,ON");     /* Ashtech TPV solution */
        monitor_nmea_send("$PASHS,NME,SAT,A,ON");   // Ashtech Satellite status
        monitor_nmea_send("$PASHS,NME,MCA,A,ON");     /* MCA measurements */
        monitor_nmea_send("$PASHS,NME,PBN,A,ON");     /* ECEF TPV solution */
        monitor_nmea_send("$PASHS,NME,SNV,A,ON,10");  /* Almanac data */

        monitor_nmea_send("$PASHS,NME,XMG,A,ON");     /* exception messages */
        break;

    default:
        return COMMAND_UNKNOWN;
    }

    return COMMAND_UNKNOWN;
}

const struct monitor_object_t ashtech_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = ashtech_command,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_ashtech,
};

#ifdef FV18_ENABLE
extern const struct gps_type_t driver_fv18;

const struct monitor_object_t fv18_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_fv18,
};
#endif /* FV18_ENABLE */

#ifdef GPSCLOCK_ENABLE
extern const struct gps_type_t driver_gpsclock;

const struct monitor_object_t gpsclock_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_gpsclock,
};
#endif /* GPSCLOCK_ENABLE */

extern const struct gps_type_t driver_mtk3301;

const struct monitor_object_t mtk3301_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_mtk3301,
};

#ifdef AIVDM_ENABLE
extern const struct gps_type_t driver_aivdm;

const struct monitor_object_t aivdm_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_aivdm,
};
#endif  // AIVDM_ENABLE

/* vim: set expandtab shiftwidth=4: */
