#include "AP_Choreo.h"

#if AP_CHOREO_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>      // mavlink_system
#include <AP_Math/AP_Math.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef O_RDONLY
#define O_RDONLY 0
#endif

// Count comma columns on a line (commas + 1 = column count)
static int _count_columns(const char *line)
{
    int n = 1;
    for (const char *p = line; *p && *p != '\n' && *p != '\r'; p++) {
        if (*p == ',') n++;
    }
    return n;
}

// Trim leading whitespace
static const char *_skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p)) p++;
    return p;
}

// Convert a 0..1 float to 0..255 uint8
static uint8_t _f_to_u8(float v)
{
    int x = int(v * 255.0f + 0.5f);
    if (x < 0)   x = 0;
    if (x > 255) x = 255;
    return uint8_t(x);
}

// Simple insertion sort by frame/t_s ordinal (n is small, MAX 500)
struct _RawWp {
    AP_Choreo::Waypoint wp;
    float order_key;        // raw frame or t_s value before normalization
};

static void _sort_by_key(_RawWp *arr, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++) {
        _RawWp tmp = arr[i];
        int16_t j = int16_t(i) - 1;
        while (j >= 0 && arr[j].order_key > tmp.order_key) {
            arr[j+1] = arr[j];
            j--;
        }
        arr[j+1] = tmp;
    }
}

bool AP_Choreo::_load_csv(const char *path)
{
    auto &fs = AP::FS();
    int fd = fs.open(path, O_RDONLY);
    if (fd < 0) {
        return false;
    }

    const uint8_t self_sysid = mavlink_system.sysid;
    _num_wps = 0;

    char line[200];
    int  header_cols = 0;       // 8 (legacy NED) or 9 (with type)
    bool header_seen = false;

    // Scratch raw buffer (so we can sort by key, then normalize to t_norm)
    static _RawWp raw[MAX_WPS];
    uint16_t raw_n = 0;

    while (fs.fgets(line, sizeof(line), fd)) {
        const char *p = _skip_ws(line);
        if (*p == '\0' || *p == '#' || *p == '\n' || *p == '\r') {
            continue;
        }

        // First non-blank, non-comment line: header (or first data row)
        if (!header_seen) {
            header_seen = true;
            header_cols = _count_columns(p);
            // If line starts with alpha (e.g. "uavid,..."), treat as header and skip.
            // Otherwise (numeric) treat as data and fall through.
            if (isalpha((unsigned char)*p)) {
                continue;
            }
        }

        // Parse a data row
        if (raw_n >= MAX_WPS) {
            break;
        }

        Waypoint wp {};
        float order_key = 0.0f;
        bool  ok = false;

        if (header_cols == 8) {
            // Legacy: uavid,frame,x,y,z,r,g,b   (Blender XYZ -> NED)
            int uavid = 0;
            int frame = 0;
            float x = 0, y = 0, z = 0, rf = 0, gf = 0, bf = 0;
            if (sscanf(p, "%d,%d,%f,%f,%f,%f,%f,%f",
                       &uavid, &frame, &x, &y, &z, &rf, &gf, &bf) == 8) {
                if (uavid == int(self_sysid)) {
                    wp.type = 'N';
                    // Blender (x,y,z) -> NED: north=y, east=x, up=z
                    wp.pos.ned.north_m = y;
                    wp.pos.ned.east_m  = x;
                    wp.pos.ned.up_m    = z;
                    wp.r = _f_to_u8(rf);
                    wp.g = _f_to_u8(gf);
                    wp.b = _f_to_u8(bf);
                    order_key = float(frame);
                    ok = true;
                }
            }
        } else if (header_cols == 9) {
            // New: uavid,frame_or_ts,type,a,b,c,r,g,b
            int uavid = 0;
            float key = 0;
            char tch = 0;
            float a = 0, b = 0, c = 0, rf = 0, gf = 0, bf = 0;
            if (sscanf(p, "%d,%f,%c,%f,%f,%f,%f,%f,%f",
                       &uavid, &key, &tch, &a, &b, &c, &rf, &gf, &bf) == 9) {
                if (uavid == int(self_sysid)) {
                    if (tch == 'L' || tch == 'l') {
                        wp.type = 'L';
                        wp.pos.lla.lat_1e7 = int32_t(a * 1e7);
                        wp.pos.lla.lng_1e7 = int32_t(b * 1e7);
                        wp.pos.lla.up_m    = c;
                    } else {
                        // Default to NED
                        wp.type = 'N';
                        wp.pos.ned.north_m = a;
                        wp.pos.ned.east_m  = b;
                        wp.pos.ned.up_m    = c;
                    }
                    wp.r = _f_to_u8(rf);
                    wp.g = _f_to_u8(gf);
                    wp.b = _f_to_u8(bf);
                    order_key = key;
                    ok = true;
                }
            }
        }

        if (ok) {
            raw[raw_n].wp = wp;
            raw[raw_n].order_key = order_key;
            raw_n++;
        }
    }

    fs.close(fd);

    // Sort by raw key (frame number or t_s) so the t_norm space is monotonic
    if (raw_n > 1) {
        _sort_by_key(raw, raw_n);
    }

    // Normalize to t_norm in [0, 1]
    if (raw_n == 0) {
        _num_wps = 0;
        gcs().send_text(MAV_SEVERITY_WARNING,
            "AP_Choreo: 0 wps for sysid=%u in %s",
            (unsigned)self_sysid, path);
        return false;
    }

    const float k_min = raw[0].order_key;
    const float k_max = raw[raw_n-1].order_key;
    const float span  = (k_max - k_min);

    for (uint16_t i = 0; i < raw_n; i++) {
        _wps[i] = raw[i].wp;
        if (span > 1e-6f) {
            _wps[i].t_norm = (raw[i].order_key - k_min) / span;
        } else {
            _wps[i].t_norm = 0.0f;
        }
    }
    _num_wps = raw_n;

    gcs().send_text(MAV_SEVERITY_INFO,
        "AP_Choreo: loaded %u wps (t_norm 0..1) for sysid=%u",
        (unsigned)_num_wps, (unsigned)self_sysid);

    return _num_wps > 0;
}

#endif // AP_CHOREO_ENABLED
