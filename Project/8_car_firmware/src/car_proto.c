#include "car_proto.h"

#include <string.h>

#define PROTO_LINE_MAX 64

static char    s_line[PROTO_LINE_MAX];
static uint8_t s_len;

void car_proto_reset(void)
{
    s_len = 0;
}

/* ---------- 极简数字解析（不引 stdlib 的 atof） ---------- */

static int parse_float(const char **pp, float *out)
{
    const char *p = *pp;
    int   neg = 0;
    float v   = 0.0f;
    float frac = 0.0f;
    float base = 0.1f;
    int   any = 0;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '-' || *p == '+') { neg = (*p == '-'); p++; }

    while (*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; any = 1; }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            frac += (float)(*p - '0') * base;
            base *= 0.1f;
            p++;
            any = 1;
        }
    }
    if (!any) return 0;

    v += frac;
    *out = neg ? -v : v;
    *pp = p;
    return 1;
}

/* ---------- 拼字符串 ---------- */

static int put_str(char *buf, int size, int pos, const char *s)
{
    while (*s && pos < size - 1) buf[pos++] = *s++;
    buf[pos] = 0;
    return pos;
}

static int put_int(char *buf, int size, int pos, long v)
{
    char tmp[12];
    int  n = 0;
    int  neg = (v < 0);
    unsigned long u = neg ? (unsigned long)(-v) : (unsigned long)v;

    if (u == 0) tmp[n++] = '0';
    while (u) { tmp[n++] = (char)('0' + u % 10u); u /= 10u; }
    if (neg) tmp[n++] = '-';
    while (n-- > 0 && pos < size - 1) buf[pos++] = tmp[n];
    buf[pos] = 0;
    return pos;
}

static long round_half_away(float v)
{
    return (long)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static int put_fixed(char *buf, int size, int pos, long v, int decimals)
{
    long div = 1;
    for (int i = 0; i < decimals; i++) div *= 10;

    if (v < 0) { if (pos < size - 1) buf[pos++] = '-'; v = -v; }
    pos = put_int(buf, size, pos, v / div);
    if (decimals > 0) {
        if (pos < size - 1) buf[pos++] = '.';
        long f = v % div;
        for (long d = div / 10; d > 0; d /= 10) {
            if (pos < size - 1) buf[pos++] = (char)('0' + (f / d) % 10);
        }
    }
    buf[pos] = 0;
    return pos;
}

/* ---------- 解析一行 ---------- */

static int parse_line(const char *line, car_cmd_t *out)
{
    out->kind = CAR_CMD_NONE;
    out->act  = 0;
    out->v    = 0.0f;
    out->w    = 0.0f;

    if (strncmp(line, "@MOVE", 5) == 0) {
        const char *p = line + 5;
        float a;
        if (!parse_float(&p, &a)) return 0;
        int act = (int)(a + 0.5f);
        if (act < 0) act = 0;
        if (act > 8) act = 0;
        out->kind = CAR_CMD_MOVE;
        out->act  = (uint8_t)act;
        return 1;
    }
    if (strncmp(line, "@VEL", 4) == 0) {
        const char *p = line + 4;
        float v, w;
        if (!parse_float(&p, &v)) return 0;
        if (!parse_float(&p, &w)) return 0;
        out->kind = CAR_CMD_VEL;
        out->v = v;
        out->w = w;
        return 1;
    }
    if (strncmp(line, "@STOP", 5) == 0) {
        out->kind = CAR_CMD_STOP;
        return 1;
    }
    if (strncmp(line, "@MODE", 5) == 0) {
        const char *p = line + 5;
        float m;
        if (!parse_float(&p, &m)) return 0;
        out->kind = CAR_CMD_MODE;
        out->mode = (m >= 0.5f) ? 1 : 0;
        return 1;
    }
    if (strncmp(line, "@CAL", 4) == 0) {
        out->kind = CAR_CMD_CAL;
        return 1;
    }
    return 0;
}

static int s_seq = -1;
int car_proto_last_seq(void) { return s_seq; }

int car_proto_fmt_ackseq(char *buf, int size, int seq)
{
    int pos = 0;
    pos = put_str(buf, size, pos, "@ACK ");
    pos = put_int(buf, size, pos, (long)seq);
    pos = put_str(buf, size, pos, "\n");
    return pos;
}

int car_proto_feed(uint8_t byte, car_cmd_t *out)
{
    if (byte == '\r') return 0;

    if (byte == '\n') {
        s_line[s_len] = 0;
        /*: 无线链路：行尾 "#<seq>" 拆出来，正文照旧解析；没有 '#' 时 seq = -1，
         *  行为与原来的 @MOVE/@VEL 完全一致（基线不受影响）。 */
        s_seq = -1;
        char *hash = 0;
        for (int i = 0; i < s_len; ++i) if (s_line[i] == '#') { hash = &s_line[i]; break; }
        if (hash) {
            *hash = 0;
            int v = 0, any = 0;
            for (char *q = hash + 1; *q; ++q) {
                if (*q < '0' || *q > '9') break;
                v = v * 10 + (*q - '0'); any = 1;
            }
            if (any) s_seq = v;
        }
        int ok = (s_len > 0) ? parse_line(s_line, out) : 0;
        s_len = 0;
        return ok;
    }

    if (s_len < PROTO_LINE_MAX - 1) s_line[s_len++] = (char)byte;
    else s_len = 0;                       /* 行太长，丢掉重来 */
    return 0;
}

/* ---------- 输出 ---------- */

int car_proto_fmt_odom(char *buf, int size, car_pose_t p)
{
    int pos = 0;
    pos = put_str(buf, size, pos, "@ODOM ");
    pos = put_int(buf, size, pos, round_half_away(p.x * 1000.0f));   /* mm */
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, round_half_away(p.y * 1000.0f));
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, round_half_away(p.theta * 1000.0f)); /* mrad */
    pos = put_str(buf, size, pos, "\n");
    return pos;
}

int car_proto_fmt_state(char *buf, int size, float angle_cdeg,
                        float v_mm_s, float w_mrad_s, uint8_t act)
{
    int pos = 0;
    pos = put_str(buf, size, pos, "@STATE ");
    pos = put_fixed(buf, size, pos, round_half_away(angle_cdeg), 1);
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, round_half_away(v_mm_s));
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, round_half_away(w_mrad_s));
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, (long)act);
    pos = put_str(buf, size, pos, "\n");
    return pos;
}

int car_proto_fmt_ack(char *buf, int size, const char *name, float v, float w)
{
    int pos = 0;
    pos = put_str(buf, size, pos, "@ACK ");
    pos = put_str(buf, size, pos, name);
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, (long)v);
    pos = put_str(buf, size, pos, " ");
    pos = put_int(buf, size, pos, (long)w);
    pos = put_str(buf, size, pos, "\n");
    return pos;
}
