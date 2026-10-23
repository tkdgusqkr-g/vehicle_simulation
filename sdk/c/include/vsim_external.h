/* vsim_external.h - the lockstep link of external components (design doc §11, §12.5).
 *
 * An external component is a separate program - a virtual ECU running real
 * firmware in an MCU emulator (Renode, QEMU), a test bench, a controller in
 * another language - that vsim drives step by step over TCP. vsim listens
 * (components.<name>: {type: external, listen: "127.0.0.1:0", command: [...]})
 * and the program connects to the address in the VSIM_EXTERNAL environment
 * variable (host:port, set when vsim starts the program itself).
 *
 * Messages: uint32 length (of what follows), uint8 type, payload. Numbers
 * are little-endian; doubles are IEEE 754.
 *
 *   HELLO  (program -> vsim)  JSON {"protocol": 1, "name": "...", "version": "...",
 *                                   "ports": [{"name": "throttle", "unit": "-", "dir": "in"}, ...]}
 *   SETUP  (vsim -> program)  JSON {"protocol": 1, "component": "...", "period": 0.001,
 *                                   "params": {...}, "inputs": [...], "outputs": [...], "buses": [...]}
 *   STEP   (vsim -> program)  f64 t, u32 n_in, f64 in[n_in], u32 n_frames, frames
 *   RESULT (program -> vsim)  u32 n_out, f64 out[n_out], u32 n_frames, frames
 *   END    (vsim -> program)  (empty) the run is over; close and exit
 *
 * Inputs and outputs are in port order (inputs then outputs as declared in
 * HELLO), in the units declared. A frame is u8 bus (index into SETUP's
 * buses), u8 flags (1 extended id, 2 CAN FD, 4 bit rate switch), u32 id,
 * u8 length, length data bytes. STEP carries the frames the component
 * received on the virtual buses since its last step; RESULT the frames it
 * transmits now. Every STEP must be answered before vsim goes on.
 *
 * C client (define VSIM_EXTERNAL_IMPLEMENTATION in one file):
 *
 *   VsimExternal link;
 *   VsimExternalPort ports[] = {{"throttle", "-", 1}, {"torque", "N*m", 0}};
 *   if (vsim_external_connect(&link, NULL, "my_ecu", "1.0", ports, 2) != 0) return 1;
 *   double t, in[1], out[1];
 *   while (vsim_external_next(&link, &t, in) == 1) {      // 0: END, -1: error
 *     out[0] = 230.0 * in[0];
 *     if (vsim_external_reply(&link, out) != 0) return 1;
 *   }
 *   vsim_external_close(&link);
 */
#ifndef VSIM_EXTERNAL_H
#define VSIM_EXTERNAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VSIM_EXTERNAL_PROTOCOL 1

enum {
  VSIM_EXT_HELLO = 1,
  VSIM_EXT_SETUP = 2,
  VSIM_EXT_STEP = 3,
  VSIM_EXT_RESULT = 4,
  VSIM_EXT_END = 5
};

enum { VSIM_FRAME_EXTENDED = 1, VSIM_FRAME_FD = 2, VSIM_FRAME_BRS = 4 };

typedef struct VsimExternalPort {
  const char *name;
  const char *unit;
  int input; /* 1 input, 0 output */
} VsimExternalPort;

typedef struct VsimExternalFrame {
  uint8_t bus;
  uint8_t flags;
  uint32_t id;
  uint8_t length;
  uint8_t data[64];
} VsimExternalFrame;

#define VSIM_EXTERNAL_MAX_FRAMES 256

typedef struct VsimExternal {
  intptr_t socket;
  int inputs, outputs;
  char setup[8192]; /* SETUP JSON as received */
  /* Frames received with the last STEP / to send with the next RESULT. */
  VsimExternalFrame rx[VSIM_EXTERNAL_MAX_FRAMES];
  int rx_count;
  VsimExternalFrame tx[VSIM_EXTERNAL_MAX_FRAMES];
  int tx_count;
} VsimExternal;

/* Connects to `address` ("host:port"; NULL: $VSIM_EXTERNAL), sends HELLO and
 * reads SETUP. Returns 0 on success. */
int vsim_external_connect(VsimExternal *link, const char *address, const char *name, const char *version,
                          const VsimExternalPort *ports, int port_count);
/* Waits for the next STEP: 1 with *t and in[] filled (and link->rx), 0 at END, -1 on error. */
int vsim_external_next(VsimExternal *link, double *t, double *in);
/* Answers the STEP with out[] and the frames queued in link->tx (then clears them). 0 on success. */
int vsim_external_reply(VsimExternal *link, const double *out);
/* Queues a frame for the next reply. 0 on success, -1 if the queue is full. */
int vsim_external_send_frame(VsimExternal *link, const VsimExternalFrame *frame);
void vsim_external_close(VsimExternal *link);

#ifdef __cplusplus
}
#endif

#ifdef VSIM_EXTERNAL_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET vsim_socket_t;
#define VSIM_CLOSE_SOCKET closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int vsim_socket_t;
#define VSIM_CLOSE_SOCKET close
#endif

static int vsim_ext_send_all(VsimExternal *link, const void *data, size_t size) {
  const char *p = (const char *)data;
  while (size > 0) {
    int n = (int)send((vsim_socket_t)link->socket, p, (int)size, 0);
    if (n <= 0) return -1;
    p += n;
    size -= (size_t)n;
  }
  return 0;
}

static int vsim_ext_recv_all(VsimExternal *link, void *data, size_t size) {
  char *p = (char *)data;
  while (size > 0) {
    int n = (int)recv((vsim_socket_t)link->socket, p, (int)size, 0);
    if (n <= 0) return -1;
    p += n;
    size -= (size_t)n;
  }
  return 0;
}

/* Little-endian packing (also correct on big-endian hosts). */
static void vsim_ext_put_u32(unsigned char *p, uint32_t v) {
  p[0] = (unsigned char)v; p[1] = (unsigned char)(v >> 8); p[2] = (unsigned char)(v >> 16); p[3] = (unsigned char)(v >> 24);
}
static uint32_t vsim_ext_get_u32(const unsigned char *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void vsim_ext_put_f64(unsigned char *p, double v) {
  uint64_t bits;
  int i;
  memcpy(&bits, &v, 8);
  for (i = 0; i < 8; ++i) p[i] = (unsigned char)(bits >> (8 * i));
}
static double vsim_ext_get_f64(const unsigned char *p) {
  uint64_t bits = 0;
  double v;
  int i;
  for (i = 0; i < 8; ++i) bits |= (uint64_t)p[i] << (8 * i);
  memcpy(&v, &bits, 8);
  return v;
}

static int vsim_ext_send_message(VsimExternal *link, int type, const unsigned char *payload, uint32_t size) {
  unsigned char head[5];
  vsim_ext_put_u32(head, size + 1);
  head[4] = (unsigned char)type;
  if (vsim_ext_send_all(link, head, 5) != 0) return -1;
  return size > 0 ? vsim_ext_send_all(link, payload, size) : 0;
}

/* Reads one message into a malloc'ed buffer (*payload, freed by the caller). */
static int vsim_ext_read_message(VsimExternal *link, int *type, unsigned char **payload, uint32_t *size) {
  unsigned char head[5];
  if (vsim_ext_recv_all(link, head, 5) != 0) return -1;
  *size = vsim_ext_get_u32(head) - 1;
  *type = head[4];
  *payload = (unsigned char *)malloc(*size + 1);
  if (*payload == NULL) return -1;
  (*payload)[*size] = 0;
  return *size > 0 ? vsim_ext_recv_all(link, *payload, *size) : 0;
}

int vsim_external_connect(VsimExternal *link, const char *address, const char *name, const char *version,
                          const VsimExternalPort *ports, int port_count) {
  char host[128] = "127.0.0.1", json[8192];
  int port = 0, i, type, n;
  const char *colon;
  struct sockaddr_in addr;
  vsim_socket_t s;
  unsigned char *payload = NULL;
  uint32_t size = 0;
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  memset(link, 0, sizeof *link);
  if (address == NULL) address = getenv("VSIM_EXTERNAL");
  if (address == NULL) return -1;
  colon = strrchr(address, ':');
  if (colon == NULL || (size_t)(colon - address) >= sizeof host) return -1;
  memcpy(host, address, (size_t)(colon - address));
  host[colon - address] = 0;
  port = atoi(colon + 1);
  s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, host, &addr.sin_addr) != 1 || connect(s, (struct sockaddr *)&addr, sizeof addr) != 0) {
    VSIM_CLOSE_SOCKET(s);
    return -1;
  }
  i = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&i, sizeof i);
  link->socket = (intptr_t)s;
  n = snprintf(json, sizeof json, "{\"protocol\": %d, \"name\": \"%s\", \"version\": \"%s\", \"ports\": [",
               VSIM_EXTERNAL_PROTOCOL, name, version);
  for (i = 0; i < port_count && n < (int)sizeof json; ++i) {
    n += snprintf(json + n, sizeof json - (size_t)n, "%s{\"name\": \"%s\", \"unit\": \"%s\", \"dir\": \"%s\"}",
                  i ? ", " : "", ports[i].name, ports[i].unit ? ports[i].unit : "", ports[i].input ? "in" : "out");
    if (ports[i].input) link->inputs++; else link->outputs++;
  }
  n += snprintf(json + n, sizeof json - (size_t)n, "]}");
  if (n >= (int)sizeof json || vsim_ext_send_message(link, VSIM_EXT_HELLO, (const unsigned char *)json, (uint32_t)n) != 0)
    return -1;
  if (vsim_ext_read_message(link, &type, &payload, &size) != 0 || type != VSIM_EXT_SETUP) {
    free(payload);
    return -1;
  }
  strncpy(link->setup, (const char *)payload, sizeof link->setup - 1);
  free(payload);
  return 0;
}

int vsim_external_next(VsimExternal *link, double *t, double *in) {
  int type, i;
  unsigned char *payload = NULL, *p;
  uint32_t size = 0, n_in, n_frames;
  if (vsim_ext_read_message(link, &type, &payload, &size) != 0) {
    free(payload);
    return -1;
  }
  if (type == VSIM_EXT_END) {
    free(payload);
    return 0;
  }
  if (type != VSIM_EXT_STEP || size < 12) {
    free(payload);
    return -1;
  }
  p = payload;
  *t = vsim_ext_get_f64(p);
  n_in = vsim_ext_get_u32(p + 8);
  p += 12;
  for (i = 0; i < (int)n_in; ++i, p += 8) {
    if (i < link->inputs) in[i] = vsim_ext_get_f64(p);
  }
  n_frames = vsim_ext_get_u32(p);
  p += 4;
  link->rx_count = 0;
  for (i = 0; i < (int)n_frames; ++i) {
    VsimExternalFrame f;
    f.bus = p[0];
    f.flags = p[1];
    f.id = vsim_ext_get_u32(p + 2);
    f.length = p[6] > 64 ? 64 : p[6];
    memcpy(f.data, p + 7, f.length);
    p += 7 + p[6];
    if (link->rx_count < VSIM_EXTERNAL_MAX_FRAMES) link->rx[link->rx_count++] = f;
  }
  free(payload);
  return 1;
}

int vsim_external_send_frame(VsimExternal *link, const VsimExternalFrame *frame) {
  if (link->tx_count >= VSIM_EXTERNAL_MAX_FRAMES) return -1;
  link->tx[link->tx_count++] = *frame;
  return 0;
}

int vsim_external_reply(VsimExternal *link, const double *out) {
  size_t size = 8 + 8 * (size_t)link->outputs + (size_t)link->tx_count * (7 + 64);
  unsigned char *buf = (unsigned char *)malloc(size), *p = buf;
  int i, status;
  if (buf == NULL) return -1;
  vsim_ext_put_u32(p, (uint32_t)link->outputs);
  p += 4;
  for (i = 0; i < link->outputs; ++i, p += 8) vsim_ext_put_f64(p, out[i]);
  vsim_ext_put_u32(p, (uint32_t)link->tx_count);
  p += 4;
  for (i = 0; i < link->tx_count; ++i) {
    const VsimExternalFrame *f = &link->tx[i];
    p[0] = f->bus;
    p[1] = f->flags;
    vsim_ext_put_u32(p + 2, f->id);
    p[6] = f->length > 64 ? 64 : f->length;
    memcpy(p + 7, f->data, p[6]);
    p += 7 + p[6];
  }
  link->tx_count = 0;
  status = vsim_ext_send_message(link, VSIM_EXT_RESULT, buf, (uint32_t)(p - buf));
  free(buf);
  return status;
}

void vsim_external_close(VsimExternal *link) {
  if (link->socket) VSIM_CLOSE_SOCKET((vsim_socket_t)link->socket);
  link->socket = 0;
}

#endif /* VSIM_EXTERNAL_IMPLEMENTATION */
#endif /* VSIM_EXTERNAL_H */
