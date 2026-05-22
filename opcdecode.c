#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <zstd.h>
#include <signal.h>

#define HEADER_SIZE         32
#define COMP_BLOCK_SIZE      8
#define SPARSE_RANGE_SIZE    6

/* ── timing ──────────────────────────────────────────────────────────────── */

static double current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* ── networking ──────────────────────────────────────────────────────────── */

static int connect_opc(const char *ip, int port) {
    struct sockaddr_in server = {
        .sin_family = AF_INET,
        .sin_port   = htons(port),
    };
    inet_pton(AF_INET, ip, &server.sin_addr);

    while (1) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { perror("socket"); exit(1); }

        if (connect(sock, (struct sockaddr *)&server, sizeof(server)) == 0)
            return sock;

        perror("connect failed, retrying in 5s");
        close(sock);
        sleep(5);
    }
}

static void send_pixels(int sock, const uint8_t *data, uint32_t length) {
    uint8_t header[4] = {
        0x00,                        /* channel  */
        0x00,                        /* command  */
        (length >> 8) & 0xFF,
        length & 0xFF,
    };
    send(sock, header, 4, 0);
    send(sock, data, length, 0);
}

/* ── FSEQ structures ─────────────────────────────────────────────────────── */

typedef struct {
    uint32_t first_frame;
    uint32_t size;
} CompBlock;

typedef struct {
    uint32_t start_channel;   /* 24-bit field, expanded here */
    uint32_t channel_count;   /* 24-bit field, expanded here */
} SparseRange;

typedef struct {
    uint16_t channel_data_offset;
    uint8_t  minor_version;
    uint8_t  major_version;
    uint32_t channel_count;
    uint32_t frame_count;
    uint8_t  step_time_ms;
    uint8_t  compression_type;   /* 0=none, 1=zstd, 2=zlib */
    uint16_t comp_block_count;   /* up to 12 bits (extended) */
    uint8_t  sparse_range_count;

    CompBlock   *comp_blocks;
    SparseRange *sparse_ranges;
} FSEQHeader;

/* Read a 24-bit little-endian integer from a byte pointer. */
static uint32_t read_u24_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

/* ── header parsing ──────────────────────────────────────────────────────── */

static int parse_header(FILE *f, FSEQHeader *h) {
    uint8_t raw[HEADER_SIZE];
    if (fread(raw, 1, HEADER_SIZE, f) != HEADER_SIZE) return 0;

    if (memcmp(raw, "PSEQ", 4) != 0 && memcmp(raw, "FSEQ", 4) != 0) {
        fprintf(stderr, "Not a valid FSEQ file\n");
        return 0;
    }

    h->channel_data_offset = *(uint16_t *)(raw + 4);
    h->minor_version       = raw[6];
    h->major_version       = raw[7];

    /* bytes 10-13: channel count */
    h->channel_count = *(uint32_t *)(raw + 10);
    /* bytes 14-17: frame count  */
    h->frame_count   = *(uint32_t *)(raw + 14);
    /* byte 18: step time        */
    h->step_time_ms  = raw[18];

    /*
     * Byte 20 encodes two fields:
     *   lower nibble → compression type  (0=none, 1=zstd, 2=zlib)
     *   upper nibble → extended compression block count (high 4 bits of the
     *                  12-bit block count)
     * Byte 21       → lower 8 bits of compression block count
     */
    uint8_t ct_byte  = raw[20];
    h->compression_type = ct_byte & 0x0F;
    uint8_t ecbc        = (ct_byte >> 4) & 0x0F;   /* upper 4 bits */
    h->comp_block_count = ((uint16_t)ecbc << 8) | raw[21];
    h->sparse_range_count = raw[22];

    printf("FSEQ Version     : %u.%u\n", h->major_version, h->minor_version);
    printf("Channel Count    : %u\n",    h->channel_count);
    printf("Frame Count      : %u\n",    h->frame_count);
    printf("Step Time        : %u ms\n", h->step_time_ms);
    printf("Compression Type : %u (%s)\n", h->compression_type,
           h->compression_type == 0 ? "none" :
           h->compression_type == 1 ? "zstd" :
           h->compression_type == 2 ? "zlib" : "unknown");
    printf("Comp Blocks      : %u\n",    h->comp_block_count);
    printf("Sparse Ranges    : %u\n",    h->sparse_range_count);

    /* Read compression blocks (8 bytes each, immediately after the 32-byte header) */
    h->comp_blocks = NULL;
    if (h->comp_block_count > 0) {
        h->comp_blocks = malloc(h->comp_block_count * sizeof(CompBlock));
        if (!h->comp_blocks) { perror("malloc comp_blocks"); return 0; }

        for (uint16_t i = 0; i < h->comp_block_count; i++) {
            uint8_t cb[COMP_BLOCK_SIZE];
            if (fread(cb, 1, COMP_BLOCK_SIZE, f) != COMP_BLOCK_SIZE) {
                fprintf(stderr, "Truncated comp block table\n");
                free(h->comp_blocks); return 0;
            }
            h->comp_blocks[i].first_frame = *(uint32_t *)(cb + 0);
            h->comp_blocks[i].size        = *(uint32_t *)(cb + 4);
        }
    }

    /* Read sparse ranges (6 bytes each, 24-bit fields) */
    h->sparse_ranges = NULL;
    if (h->sparse_range_count > 0) {
        h->sparse_ranges = malloc(h->sparse_range_count * sizeof(SparseRange));
        if (!h->sparse_ranges) { perror("malloc sparse_ranges"); free(h->comp_blocks); return 0; }

        for (uint8_t i = 0; i < h->sparse_range_count; i++) {
            uint8_t sr[SPARSE_RANGE_SIZE];
            if (fread(sr, 1, SPARSE_RANGE_SIZE, f) != SPARSE_RANGE_SIZE) {
                fprintf(stderr, "Truncated sparse range table\n");
                free(h->comp_blocks); free(h->sparse_ranges); return 0;
            }
            h->sparse_ranges[i].start_channel = read_u24_le(sr + 0);
            h->sparse_ranges[i].channel_count = read_u24_le(sr + 3);
        }
    }

    return 1;
}

static void free_header(FSEQHeader *h) {
    free(h->comp_blocks);
    free(h->sparse_ranges);
}

/* ── sparse expansion ────────────────────────────────────────────────────── */

/*
 * When sparse ranges are present the raw frame data is a concatenation of only
 * the specified channel ranges.  We expand it into a zero-initialised full
 * frame buffer (channel_count bytes) so send_pixels always sees a contiguous
 * RGB array starting at channel 0.
 *
 * Note: for a sparse file h->channel_count is the *sum* of all range counts,
 * which equals the raw frame size.  We need the full controller width to
 * allocate the output buffer; we derive that from the last range's end.
 */
static uint32_t full_channel_count(const FSEQHeader *h) {
    if (h->sparse_range_count == 0) return h->channel_count;
    uint32_t max_ch = 0;
    for (uint8_t i = 0; i < h->sparse_range_count; i++) {
        uint32_t end = h->sparse_ranges[i].start_channel
                     + h->sparse_ranges[i].channel_count;
        if (end > max_ch) max_ch = end;
    }
    return max_ch;
}

/* Copy packed sparse frame data → expanded full frame buffer. */
static void expand_sparse_frame(const FSEQHeader *h,
                                 const uint8_t *packed,
                                 uint8_t       *full)
{
    const uint8_t *src = packed;
    for (uint8_t i = 0; i < h->sparse_range_count; i++) {
        uint32_t start = h->sparse_ranges[i].start_channel;
        uint32_t count = h->sparse_ranges[i].channel_count;
        memcpy(full + start, src, count);
        src += count;
    }
}

/* ── playback: uncompressed ──────────────────────────────────────────────── */

static void play_uncompressed(FILE *f, const FSEQHeader *h, int sock) {
    fseek(f, h->channel_data_offset, SEEK_SET);

    int sparse    = (h->sparse_range_count > 0);
    uint32_t full = full_channel_count(h);

    uint8_t *frame_buf = calloc(1, h->channel_count);   /* raw read buffer  */
    uint8_t *send_buf  = sparse ? calloc(1, full) : frame_buf;

    if (!frame_buf || (sparse && !send_buf)) { perror("malloc"); return; }

    double start = current_time_ms();

    for (uint32_t fn = 0; fn < h->frame_count; fn++) {
        if (fread(frame_buf, 1, h->channel_count, f) < h->channel_count) {
            fprintf(stderr, "Incomplete frame at %u\n", fn);
            break;
        }

        if (sparse) {
            memset(send_buf, 0, full);
            expand_sparse_frame(h, frame_buf, send_buf);
            send_pixels(sock, send_buf, full);
        } else {
            send_pixels(sock, frame_buf, h->channel_count);
        }

        double target = (double)fn * h->step_time_ms;
        double now    = current_time_ms();
        while ((now - start) < target) now = current_time_ms();
    }

    free(frame_buf);
    if (sparse) free(send_buf);
}

/* ── playback: ZSTD compressed ───────────────────────────────────────────── */

/*
 * xLights/fpp strip the Zstandard frame magic, so we must use the streaming
 * decompression API with an explicit dictionary-less context rather than the
 * simple ZSTD_decompress() which expects a full frame.
 *
 * Strategy:
 *   For each non-empty compression block:
 *     1. Read the raw compressed bytes from the file.
 *     2. Decompress the whole block into a heap buffer.
 *     3. Slice out individual frames (each h->channel_count bytes) and send
 *        them with correct timing.
 */
static void play_zstd(FILE *f, const FSEQHeader *h, int sock) {
    if (h->comp_block_count == 0) {
        fprintf(stderr, "ZSTD file has no compression blocks\n");
        return;
    }

    int sparse       = (h->sparse_range_count > 0);
    uint32_t full_ch = full_channel_count(h);
    uint8_t *send_buf = sparse ? calloc(1, full_ch) : NULL;

    /* Filter out zero-size blocks (padding artefacts). */
    uint16_t valid_count = 0;
    for (uint16_t i = 0; i < h->comp_block_count; i++)
        if (h->comp_blocks[i].size > 0) valid_count++;

    long data_start = h->channel_data_offset;

    /* Accumulate relative start offsets for each valid block. */
    uint32_t *offsets = malloc(valid_count * sizeof(uint32_t));
    uint16_t *vmap    = malloc(valid_count * sizeof(uint16_t)); /* index into comp_blocks */
    if (!offsets || !vmap) { perror("malloc"); return; }

    {
        uint32_t rel = 0;
        uint16_t vi  = 0;
        for (uint16_t i = 0; i < h->comp_block_count; i++) {
            if (h->comp_blocks[i].size == 0) continue;
            offsets[vi] = rel;
            vmap[vi]    = i;
            rel += h->comp_blocks[i].size;
            vi++;
        }
    }

    ZSTD_DCtx *dctx = ZSTD_createDCtx();
    if (!dctx) { fprintf(stderr, "ZSTD_createDCtx failed\n"); goto cleanup; }

    double start = current_time_ms();

    for (uint16_t vi = 0; vi < valid_count; vi++) {
        uint16_t   bi        = vmap[vi];
        uint32_t   comp_size = h->comp_blocks[bi].size;
        uint32_t   first_fn  = h->comp_blocks[bi].first_frame;

        /* How many frames live in this block? */
        uint32_t next_first = (vi + 1 < valid_count)
                              ? h->comp_blocks[vmap[vi + 1]].first_frame
                              : h->frame_count;
        uint32_t block_frames = next_first - first_fn;

        uint32_t decomp_size = block_frames * h->channel_count;

        /* Allocate buffers. */
        uint8_t *comp_buf   = malloc(comp_size);
        uint8_t *decomp_buf = malloc(decomp_size);
        if (!comp_buf || !decomp_buf) { perror("malloc block"); free(comp_buf); free(decomp_buf); break; }

        /* Seek and read the compressed block. */
        if (fseek(f, data_start + offsets[vi], SEEK_SET) != 0) {
            perror("fseek block"); free(comp_buf); free(decomp_buf); break;
        }
        if (fread(comp_buf, 1, comp_size, f) != comp_size) {
            fprintf(stderr, "Short read on block %u\n", vi);
            free(comp_buf); free(decomp_buf); break;
        }

        /*
         * Decompress. xLights/fpp omit the ZSTD frame header, so we use
         * ZSTD_decompressBegin / ZSTD_decompressContinue (raw block API).
         *
         * ZSTD_decompressContinue consumes one "block" at a time (in the
         * zstd internal sense).  We loop until the full compressed input is
         * consumed.
         */
        ZSTD_DCtx_reset(dctx, ZSTD_reset_session_only);
        if (ZSTD_isError(ZSTD_decompressBegin(dctx))) {
            fprintf(stderr, "ZSTD_decompressBegin error\n");
            free(comp_buf); free(decomp_buf); break;
        }

        size_t in_pos  = 0;
        size_t out_pos = 0;
        int decomp_ok  = 1;

        while (in_pos < comp_size) {
            size_t block_size = ZSTD_nextSrcSizeToDecompress(dctx);
            if (block_size == 0) break;  /* done */
            if (ZSTD_isError(block_size)) {
                fprintf(stderr, "ZSTD_nextSrcSizeToDecompress: %s\n",
                        ZSTD_getErrorName(block_size));
                decomp_ok = 0; break;
            }
            if (in_pos + block_size > comp_size) {
                fprintf(stderr, "ZSTD block overrun at block %u\n", vi);
                decomp_ok = 0; break;
            }

            size_t produced = ZSTD_decompressContinue(
                dctx,
                decomp_buf + out_pos, decomp_size - out_pos,
                comp_buf   + in_pos,  block_size);

            if (ZSTD_isError(produced)) {
                fprintf(stderr, "ZSTD_decompressContinue: %s\n",
                        ZSTD_getErrorName(produced));
                decomp_ok = 0; break;
            }
            in_pos  += block_size;
            out_pos += produced;
        }

        if (!decomp_ok) { free(comp_buf); free(decomp_buf); break; }

        /* Dispatch individual frames with timing. */
        for (uint32_t i = 0; i < block_frames; i++) {
            uint32_t  fn         = first_fn + i;
            uint8_t  *frame_data = decomp_buf + (size_t)i * h->channel_count;

            if (sparse) {
                memset(send_buf, 0, full_ch);
                expand_sparse_frame(h, frame_data, send_buf);
                send_pixels(sock, send_buf, full_ch);
            } else {
                send_pixels(sock, frame_data, h->channel_count);
            }

            double target = (double)fn * h->step_time_ms;
            double now    = current_time_ms();
            while ((now - start) < target) now = current_time_ms();
        }

        free(comp_buf);
        free(decomp_buf);
    }

cleanup:
    ZSTD_freeDCtx(dctx);
    free(offsets);
    free(vmap);
    if (send_buf) free(send_buf);
}

/* ── top-level playback dispatcher ──────────────────────────────────────── */

static void decode_fseq(const char *path, int sock) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror("fopen"); return; }

    FSEQHeader h = {0};
    if (!parse_header(f, &h)) { fclose(f); return; }

    switch (h.compression_type) {
        case 0:
            printf("Playing uncompressed FSEQ\n");
            play_uncompressed(f, &h, sock);
            break;
        case 1:
            printf("Playing ZSTD-compressed FSEQ\n");
            play_zstd(f, &h, sock);
            break;
        case 2:
            fprintf(stderr, "zlib compression is not supported\n");
            break;
        default:
            fprintf(stderr, "Unknown compression type: %u\n", h.compression_type);
    }

    free_header(&h);
    fclose(f);
}

/* ── entry point ─────────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    signal(SIGPIPE, SIG_IGN);
    const char *ip = "127.0.0.1";
    int port = 7890;

    if (argc >= 2) {
        char *sep = strchr(argv[1], ':');
        if (sep) { *sep = '\0'; ip = argv[1]; port = atoi(sep + 1); }
    }

    const char *fseq_path = (argc >= 3) ? argv[2] : "default.fseq";

    printf("Connecting to %s:%d\n", ip, port);
    int sock = connect_opc(ip, port);

    while (1) {
       decode_fseq(fseq_path, sock);
    }

    close(sock);
    return 0;
}

