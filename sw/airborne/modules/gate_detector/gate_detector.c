/*
 * gate_detector.c
 *
 * Blue post-pair gate detector for Paparazzi — pure C, no OpenCV.
 * Processes YUV422 (UYVY) camera images directly.
 *
 * ── Algorithm overview ────────────────────────────────────────────────────────
 *
 *  Each video frame goes through four stages:
 *
 *  1. COLUMN SCAN  — scan every pixel; build per-column statistics:
 *       col_count[x]  = number of blue pixels in column x
 *       col_u_sum[x]  = sum of U values for those pixels (for mean_U)
 *       col_ytop[x]   = topmost row with blue in column x
 *       col_ybot[x]   = bottommost row with blue in column x
 *
 *  2. POST FINDER  — group adjacent blue columns into vertical blob candidates:
 *       · Reject groups that fail aspect, height, or column-density criteria.
 *       · Column-density filter:  col_count[cx] / height ≥ 0.18
 *           Real posts: 0.74–0.96   Shelf/tarp edge artefacts: 0.05–0.12
 *           This is the C equivalent of the Hough column-density filter in
 *           the Python version — no Hough transform needed.
 *       · Color purity hard veto: mean_U < 120 → orange/cream pillar → reject.
 *
 *  3. PAIR SCORER  — test all O(n²) candidate pairs:
 *       Hard vetoes (fast eliminators):
 *         · gap < 1.5 × max_post_width          (single-pole split)
 *         · opening aspect > 2.2                (wide blue banner)
 *         · vertical overlap < 20 %             (unrelated blobs)
 *         · mean_U of either post < 120         (orange/cream pillar)
 *         · blue isolation score < 0.30         (continuous blue banner)
 *       Soft scoring (8 terms, sum to 1.0):
 *         height symmetry   0.16   how similar are the two post heights?
 *         overlap ratio     0.12   how much do posts overlap vertically?
 *         gap ratio         0.10   gap/avg_post_height centred on 0.70
 *         opening aspect    0.10   how square is the opening?
 *         color purity      0.16   mean_U relative to nominal blue
 *         profile contrast  0.14   post | dark gap | post scanline
 *         isolation         0.08   no blue outside post edges
 *         checker bar       0.14   alternating B/W pattern above posts
 *
 *  4. OUTPUT  — publish via VISUAL_DETECTION ABI; update gate_latest struct.
 *
 * ── False-positive suppression (ported from Python gate_detector.py) ──────────
 *
 *  All of the key suppressors from the Python version are present here:
 *  · min_vertical_aspect = 1.5          kills square furniture blobs
 *  · max_post_aspect = 14               kills person legs / thin rods
 *  · col_density_min = 0.18             kills Hough-from-shelf-edge artefacts
 *  · color_purity_hard_floor (mean_U)   kills orange / cream pillars
 *  · isolation_hard_floor = 0.30        kills continuous blue banners
 *  · max_opening_aspect = 2.2           kills wide banners
 *  · min_gap_to_post_width = 1.5        kills single-pole splits
 *  · max_mask_density = 0.28            kills flooded-with-blue frames
 *
 * ── Thread safety ─────────────────────────────────────────────────────────────
 *
 *  The video callback runs in the video thread; the periodic function runs in
 *  the autopilot thread.  Shared state is protected by gate_mutex.  The
 *  periodic function copies _gate_shared → gate_latest under the lock, then
 *  releases immediately — the autopilot thread never holds the lock long.
 */

#include "gate_detector.h"

/* Paparazzi computer-vision infrastructure */
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

/* ABI messaging (publishes VISUAL_DETECTION) */
#include "modules/core/abi.h"

/* POSIX threading (mutex for video ↔ autopilot communication) */
#include "pthread.h"

#include <string.h>   /* memcpy, memset */
#include <stdio.h>    /* printf (verbose mode only) */

/* ─────────────────────────────────────────────────────────────────────────────
 * Compile-time limits
 * ────────────────────────────────────────────────────────────────────────────*/

#define GATE_MAX_POSTS   14    /* max vertical blob candidates per frame      */
#define GATE_COL_BUF     640   /* max supported image width                   */

/* ─────────────────────────────────────────────────────────────────────────────
 * Macros
 * ────────────────────────────────────────────────────────────────────────────*/

#define GATE_CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define GATE_ABSF(x)           ((x) < 0.0f ? -(x) : (x))
#define GATE_MAX2(a, b)        ((a) > (b) ? (a) : (b))
#define GATE_MIN2(a, b)        ((a) < (b) ? (a) : (b))

/* ─────────────────────────────────────────────────────────────────────────────
 * GCS-settable parameters (also exported via gate_detector.h)
 * ────────────────────────────────────────────────────────────────────────────*/

uint8_t gate_u_min          = GATE_U_MIN;
uint8_t gate_v_max          = GATE_V_MAX;
float   gate_min_confidence = GATE_MIN_CONFIDENCE;
uint8_t gate_purity_u_floor = GATE_PURITY_U_HARD_FLOOR;
float   gate_col_density_min = GATE_COL_DENSITY_MIN;
uint8_t gate_debug_overlay  = 0;

void gate_detector_set_u_min(uint8_t val) { gate_u_min = val; }
void gate_detector_set_v_max(uint8_t val) { gate_v_max = val; }

/* ─────────────────────────────────────────────────────────────────────────────
 * Internal post struct (not exposed; gate_detection_t is the public output)
 * ────────────────────────────────────────────────────────────────────────────*/

typedef struct {
  int16_t x0, x1;   /* column span [x0, x1] inclusive              */
  int16_t y0, y1;   /* row span    [y0, y1] inclusive               */
  int16_t w, h;     /* width = x1-x0+1,  height = y1-y0+1          */
  int16_t cx;       /* centre column = (x0+x1)/2                   */
  uint8_t mean_u;   /* mean U of blue pixels in column cx           */
  float   fill;     /* col_count[cx] / h  (≥ col_density_min req'd) */
} GPost;

/* ─────────────────────────────────────────────────────────────────────────────
 * Column-scan work buffers (static to avoid large stack allocations)
 * ────────────────────────────────────────────────────────────────────────────*/

static uint16_t _col_cnt [GATE_COL_BUF];   /* blue pixel count per column      */
static uint32_t _col_usum[GATE_COL_BUF];   /* sum of U for blue pixels         */
static uint16_t _col_ytop[GATE_COL_BUF];   /* topmost blue row per column      */
static uint16_t _col_ybot[GATE_COL_BUF];   /* bottommost blue row per column   */

/* Reused by _checker_score to avoid stack allocation of large arrays */
static uint32_t _checker_ysum[GATE_COL_BUF];

/* ─────────────────────────────────────────────────────────────────────────────
 * Thread state
 * ────────────────────────────────────────────────────────────────────────────*/

static pthread_mutex_t      _gate_mutex;
static struct gate_detection_t _gate_shared;   /* updated by video thread      */
struct gate_detection_t        gate_latest;    /* copied by periodic           */
static struct video_listener  *_listener = NULL;

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 1 — Pixel helpers
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * is_blue_yuv — test whether a UYVY pixel is within the blue gate range.
 *
 * UYVY layout per pixel pair (4 bytes total):
 *   byte 0: U  (chroma: B−Y offset, 128 = neutral, >128 = blue-ish)
 *   byte 1: Y0 (luma for even column)
 *   byte 2: V  (chroma: R−Y offset, 128 = neutral, >128 = red-ish)
 *   byte 3: Y1 (luma for odd column)
 *
 * Both pixels in the pair share the same U and V.
 */
static inline bool is_blue_yuv(uint8_t U, uint8_t Y, uint8_t V)
{
  return (U > gate_u_min)   &&   /* blue dominant                 */
         (V < gate_v_max)   &&   /* not red / orange              */
         (Y > GATE_Y_MIN)   &&   /* not too dark                  */
         (Y < GATE_Y_MAX);       /* not overexposed               */
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 2 — Column scan  (Stage 1 of the pipeline)
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * gate_scan_columns — single O(w×h) pass that fills the four _col_* arrays.
 *
 * Processes pixel pairs at a time (U and V are shared for each pair) which
 * avoids redundant byte reads.  The bottom 5 % of the frame is suppressed to
 * reject floor-mat and ground-plane clutter.
 *
 * Returns: total number of blue pixels (used for global density gate).
 */
static uint32_t gate_scan_columns(const uint8_t *buf, uint16_t w, uint16_t h)
{
  if (w > GATE_COL_BUF) w = GATE_COL_BUF;   /* safety clamp */

  /* Initialise column accumulators */
  for (uint16_t x = 0; x < w; x++) {
    _col_cnt [x] = 0;
    _col_usum[x] = 0;
    _col_ytop[x] = h;   /* sentinel: "no blue found yet" */
    _col_ybot[x] = 0;
  }

  const uint16_t h_clip  = (uint16_t)((uint32_t)h * 95u / 100u);
  const uint16_t stride  = (uint16_t)(w * 2u);   /* bytes per row (UYVY = 2 B/px) */
  uint32_t total_blue = 0;

  for (uint16_t y = 0; y < h_clip; y++) {
    const uint8_t *row = buf + (uint32_t)y * stride;
    uint16_t k = 0;                 /* pair index; pair k covers columns 2k, 2k+1 */

    for (uint16_t x = 0; x < w; x += 2, k++) {
      const uint8_t *pair = row + k * 4u;
      uint8_t U  = pair[0];
      uint8_t Y0 = pair[1];
      uint8_t V  = pair[2];
      uint8_t Y1 = pair[3];

      /* Left pixel of pair (even column x) */
      if (is_blue_yuv(U, Y0, V)) {
        _col_cnt [x]++;
        _col_usum[x] += U;
        total_blue++;
        if (y < _col_ytop[x]) _col_ytop[x] = y;
        if (y > _col_ybot[x]) _col_ybot[x] = y;
      }

      /* Right pixel of pair (odd column x+1) */
      if ((x + 1u) < w && is_blue_yuv(U, Y1, V)) {
        _col_cnt [x + 1]++;
        _col_usum[x + 1] += U;
        total_blue++;
        if (y < _col_ytop[x + 1]) _col_ytop[x + 1] = y;
        if (y > _col_ybot[x + 1]) _col_ybot[x + 1] = y;
      }
    }
  }

  return total_blue;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 3 — Post finder  (Stage 2 of the pipeline)
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * gate_find_posts — convert column statistics into GPost candidates.
 *
 * Algorithm:
 *   Scan left-to-right looking for runs of "blue columns" (col_cnt ≥ threshold).
 *   For each run [x0, x1]:
 *     · Find the overall y_top / y_bot across the run.
 *     · Compute aspect ratio h/w and fill = col_cnt[cx] / h.
 *     · Apply all geometry and color-purity filters.
 *   At most max_posts candidates are returned.
 *
 * Returns: number of valid posts found.
 */
static uint8_t gate_find_posts(GPost *posts, uint8_t max_posts,
                                uint16_t w, uint16_t h)
{
  uint8_t n = 0;

  /* Minimum number of blue pixels a column must have to count as "blue".
   * Using 50 % of col_density_min so a fragmented column still qualifies
   * when computing the group extent; the stricter fill check runs later. */
  const uint16_t col_thresh = (uint16_t)(gate_col_density_min * 0.5f * (float)h);
  const int16_t  min_post_h = GATE_MAX2(4, (int16_t)((float)h * GATE_POST_MIN_HEIGHT_FRAC));
  const int16_t  max_post_w = (int16_t)(w / 5);   /* posts ≤ 20 % of frame width */

  int16_t grp_start = -1;   /* column index where the current run began */

  /* Iterate one column past w so we always close the final group */
  for (int16_t x = 0; x <= (int16_t)w; x++) {
    bool blue_col = (x < (int16_t)w) && (_col_cnt[x] >= col_thresh);

    if (blue_col && grp_start < 0) {
      grp_start = x;   /* start new group */
      continue;
    }

    if (!blue_col && grp_start >= 0) {
      /* End of group: [grp_start, x-1] */
      int16_t x0 = grp_start, x1 = x - 1;
      grp_start = -1;

      int16_t gw  = x1 - x0 + 1;
      int16_t cx  = (int16_t)((x0 + x1) / 2);

      /* ── Geometry pre-filters ──────────────────────────────────────── */
      if (gw > max_post_w) continue;   /* too wide (horizontal blob) */

      /* Compute overall y_top / y_bot across all columns in the group */
      int16_t yt = (int16_t)h, yb = 0;
      for (int16_t xx = x0; xx <= x1; xx++) {
        if (_col_cnt[xx] >= col_thresh) {
          if ((int16_t)_col_ytop[xx] < yt) yt = (int16_t)_col_ytop[xx];
          if ((int16_t)_col_ybot[xx] > yb) yb = (int16_t)_col_ybot[xx];
        }
      }
      int16_t gh = yb - yt + 1;

      if (gh < min_post_h) continue;     /* too short                    */

      float aspect = (float)gh / (float)gw;
      if (aspect < GATE_POST_MIN_ASPECT) continue;   /* square blob (furniture) */
      if (aspect > GATE_POST_MAX_ASPECT) continue;   /* thin rod / person leg   */

      /* ── Column-density filter ─────────────────────────────────────────
       * This is the C equivalent of the Python Hough col_fraction check.
       * At the centre column, the fraction of the post's vertical span that
       * actually contains blue pixels separates:
       *   Real fragmented post:  col_fraction ≈ 0.74–0.96
       *   Shelf / tarp edge:     col_fraction ≈ 0.05–0.12
       * ──────────────────────────────────────────────────────────────── */
      float fill = (gh > 0) ? ((float)_col_cnt[cx] / (float)gh) : 0.0f;
      if (fill < gate_col_density_min) continue;

      /* ── Color purity hard veto ────────────────────────────────────────
       * If the mean U value of the blue pixels in the centre column is too
       * low, this is not a blue post (orange/cream/warm pillar).
       *   Blue gate posts:     mean_U ≈ 155–220
       *   Orange pillars:      mean_U ≈ 70–100
       * ──────────────────────────────────────────────────────────────── */
      uint8_t mean_u = (_col_cnt[cx] > 0u)
                       ? (uint8_t)(_col_usum[cx] / _col_cnt[cx])
                       : 0u;
      if (mean_u < gate_purity_u_floor) continue;

      /* ── Accept ─────────────────────────────────────────────────────── */
      if (n >= max_posts) break;
      GPost *p = &posts[n++];
      p->x0     = x0;   p->x1 = x1;
      p->y0     = yt;   p->y1 = yb;
      p->w      = gw;   p->h  = gh;
      p->cx     = cx;
      p->mean_u = mean_u;
      p->fill   = fill;
    }
  }

  return n;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 4 — Scoring helpers  (Stage 3 sub-routines)
 * ═════════════════════════════════════════════════════════════════════════════*/

/* ── 4a. Blue isolation ──────────────────────────────────────────────────────
 *
 * For a real gate the blue colour is confined to the two posts.
 * For a blue banner the colour extends continuously past the outer post edges.
 * We sample the mask density in narrow windows just outside each post and
 * return 1 - max(left_extension, right_extension).
 *
 * Real gate: ≈ 0.90–1.00   Blue banner: ≈ 0.00–0.30
 */
static float gate_isolation(const GPost *lp, const GPost *rp,
                             uint16_t w, uint16_t h)
{
  int16_t opening_w = rp->x0 - lp->x1 - 1;
  if (opening_w <= 0) return 0.5f;

  int16_t check_w = GATE_MAX2(3, opening_w / 4);
  float left_ext = 0.0f, right_ext = 0.0f;
  int16_t nl = 0, nr = 0;

  for (int16_t dx = 1; dx <= check_w; dx++) {
    int16_t xl = lp->x0 - dx;
    if (xl < 0) break;
    left_ext += (lp->h > 0) ? ((float)_col_cnt[xl] / (float)lp->h) : 0.0f;
    nl++;
  }
  for (int16_t dx = 1; dx <= check_w; dx++) {
    int16_t xr = rp->x1 + dx;
    if (xr >= (int16_t)w) break;
    right_ext += (rp->h > 0) ? ((float)_col_cnt[xr] / (float)rp->h) : 0.0f;
    nr++;
  }

  if (nl > 0) left_ext  /= (float)nl;
  if (nr > 0) right_ext /= (float)nr;

  float ext = (left_ext > right_ext) ? left_ext : right_ext;
  return GATE_CLAMP(1.0f - ext, 0.0f, 1.0f);
}

/* ── 4b. Scanline profile  post | dark gap | post ───────────────────────────
 *
 * Uses per-column blue density (from the scan) at the vertical midpoint
 * of the post overlap region.  A genuine gate has high density at the posts
 * and near-zero density in the opening.
 *
 * Real gate: contrast ≈ 0.40–0.90   False positive (blue wall): ≈ 0.0–0.15
 */
static float gate_profile(const GPost *lp, const GPost *rp, uint16_t h)
{
  int16_t ov_top = GATE_MAX2(lp->y0, rp->y0);
  int16_t ov_bot = GATE_MIN2(lp->y1, rp->y1);
  if (ov_bot <= ov_top + 4) return 0.0f;

  int16_t ov_h = ov_bot - ov_top;

  /* Post density at centre columns */
  float ld = (lp->h > 0) ? ((float)_col_cnt[lp->cx] / (float)lp->h) : 0.0f;
  float rd = (rp->h > 0) ? ((float)_col_cnt[rp->cx] / (float)rp->h) : 0.0f;
  float post_d = 0.5f * (ld + rd);

  /* Gap density (average over inner columns) */
  float gap_d = 0.0f;
  int16_t ng = 0;
  for (int16_t x = lp->x1 + 1; x < rp->x0; x++) {
    gap_d += (ov_h > 0) ? ((float)_col_cnt[x] / (float)ov_h) : 0.0f;
    ng++;
  }
  if (ng > 0) gap_d /= (float)ng;

  return GATE_CLAMP((post_d - gap_d) / 0.45f, 0.0f, 1.0f);
}

/* ── 4c. Checker bar ─────────────────────────────────────────────────────────
 *
 * The competition gate has a black-and-white alternating top bar.
 * We look at the brightness (Y channel) of columns in the horizontal strip
 * just above the posts and measure the alternation using the 90th-percentile
 * of |diff(column_means)|.
 *
 * Real gate: p90 ≈ 50–150, score ≈ 0.35–1.0
 * Random objects: p90 ≈ 2–15, score ≈ 0.01–0.10
 *
 * This is a faithful C port of Python's _score_checkered_top() with the
 * p90 normalisation that avoids dilution by flat inter-square regions.
 */
static float gate_checker(const uint8_t *buf, uint16_t w, uint16_t h,
                           const GPost *lp, const GPost *rp,
                           int16_t y_top, int16_t pair_h)
{
  int16_t band_h = GATE_MAX2(4, pair_h / 15);
  int16_t y0  = GATE_MAX2(0, y_top - band_h);
  int16_t y1  = GATE_MIN2((int16_t)h, y_top + band_h / 3);
  if (y1 <= y0) return 0.0f;

  int16_t x0c = GATE_MAX2(0, lp->x0);
  int16_t x1c = GATE_MIN2((int16_t)w - 1, rp->x1);
  int16_t sw  = x1c - x0c + 1;
  if (sw < 4) return 0.0f;

  const uint16_t stride   = (uint16_t)(w * 2u);
  const uint16_t n_rows   = (uint16_t)(y1 - y0);

  /* Accumulate Y sums per column over the strip */
  for (int16_t i = 0; i < sw; i++) _checker_ysum[i] = 0;

  for (int16_t y = y0; y < y1; y++) {
    const uint8_t *row = buf + (uint32_t)y * stride;
    for (int16_t x = x0c; x <= x1c; x++) {
      uint16_t k  = (uint16_t)x >> 1u;
      uint8_t  Y  = row[k * 4u + 1u + ((uint16_t)x & 1u) * 2u];
      _checker_ysum[x - x0c] += Y;
    }
  }

  /* Compute histogram of |diff(col_means)| (range 0–255, 256 bins) */
  static uint16_t diff_hist[256];
  memset(diff_hist, 0, sizeof(diff_hist));
  uint32_t n_diffs = 0;
  uint32_t sign_changes = 0;
  int32_t  prev_mean = -1;
  int8_t   prev_sign = 0;

  for (int16_t i = 0; i < sw; i++) {
    int32_t mean = (n_rows > 0) ? (int32_t)(_checker_ysum[i] / n_rows) : 0;
    if (prev_mean >= 0) {
      int32_t d  = mean - prev_mean;
      uint8_t ad = (uint8_t)((d < 0) ? -d : d);
      diff_hist[ad]++;
      n_diffs++;
      int8_t sgn = (d > 0) ? 1 : ((d < 0) ? -1 : 0);
      if (prev_sign != 0 && sgn != 0 && sgn != prev_sign) sign_changes++;
      if (sgn != 0) prev_sign = sgn;
    }
    prev_mean = mean;
  }
  if (n_diffs == 0) return 0.0f;

  /* Find 90th percentile of |diffs| via CDF of histogram */
  uint32_t target = n_diffs * 90u / 100u;
  uint32_t cum = 0;
  float p90 = 0.0f;
  for (int i = 0; i < 256; i++) {
    cum += diff_hist[i];
    if (cum >= target) { p90 = (float)i; break; }
  }

  float alternation = 0.5f + 0.5f * (float)sign_changes
                      / (float)(n_diffs > 1u ? n_diffs - 1u : 1u);

  return GATE_CLAMP(p90 / 120.0f * alternation, 0.0f, 1.0f);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 5 — Pair scorer  (Stage 3)
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * gate_score_pair — apply all hard vetoes then compute weighted score.
 *
 * Returns the score in [0, 1], or −1.0 if a hard veto fired.
 * Caller ignores pairs with score < gate_min_confidence.
 */
static float gate_score_pair(const uint8_t *buf,
                              uint16_t w, uint16_t h,
                              const GPost *lp, const GPost *rp)
{
  /* Ensure left post is on the left */
  if (lp->cx > rp->cx) { const GPost *t = lp; lp = rp; rp = t; }

  /* ── Hard geometry vetoes ─────────────────────────────────────────────── */

  int16_t gap = rp->x0 - (lp->x1 + 1);          /* pixels between inner edges */
  if (gap < 12 || gap > (int16_t)(w * 82u / 100u)) return -1.0f;

  /* Single-pole-split suppression: gap must be ≥ 1.5 × max_post_width */
  int16_t max_pw = GATE_MAX2(lp->w, rp->w);
  if ((float)gap < GATE_MIN_GAP_TO_POST_W * (float)max_pw) return -1.0f;

  int16_t pair_h = GATE_MAX2(lp->h, rp->h);
  if (pair_h < (int16_t)((float)h * 9.0f / 100.0f)) return -1.0f;  /* too small */

  /* Opening must not be too wide (banner veto) */
  float opening_asp = (float)gap / (float)pair_h;
  if (opening_asp > GATE_MAX_OPENING_ASPECT) return -1.0f;

  /* Vertical overlap: posts must overlap to form a gate */
  int16_t ov = GATE_MIN2(lp->y1, rp->y1) - GATE_MAX2(lp->y0, rp->y0);
  int16_t min_h = GATE_MIN2(lp->h, rp->h);
  float overlap = (ov > 0 && min_h > 0) ? GATE_CLAMP((float)ov / (float)min_h, 0.0f, 1.0f) : 0.0f;
  if (overlap < 0.20f) return -1.0f;

  /* Color purity hard veto (mean_U already checked per-post in finder;
   * re-check here in case two valid single posts form a weird pair) */
  if (lp->mean_u < gate_purity_u_floor) return -1.0f;
  if (rp->mean_u < gate_purity_u_floor) return -1.0f;

  /* Blue isolation: must not extend past outer post edges */
  float iso = gate_isolation(lp, rp, w, h);
  if (iso < GATE_ISOLATION_HARD_FLOOR) return -1.0f;

  /* ── Soft scoring terms ────────────────────────────────────────────────── */

  /* 1. Height symmetry (0.16) */
  float h_ratio = (lp->h <= rp->h)
                  ? ((float)lp->h / (float)rp->h)
                  : ((float)rp->h / (float)lp->h);

  /* 2. Overlap (0.12) — already computed above */

  /* 3. Gap ratio (0.10): gap / avg_post_h centred on 0.70 */
  float avg_h      = 0.5f * (float)(lp->h + rp->h);
  float gap_ratio  = (float)gap / (avg_h > 1.0f ? avg_h : 1.0f);
  float gap_score  = GATE_CLAMP(1.0f - GATE_ABSF(gap_ratio - 0.70f) / 1.20f, 0.0f, 1.0f);

  /* 4. Opening aspect (0.10): target ≈ 1.0 (square gate) */
  float asp_score  = GATE_CLAMP(1.0f - GATE_ABSF(opening_asp - 0.90f) / 0.90f, 0.0f, 1.0f);

  /* 5. Color purity (0.16): mean_U relative to nominal gate blue (~180) */
  float purity     = GATE_CLAMP(((float)(lp->mean_u + rp->mean_u) * 0.5f - 120.0f) / 80.0f,
                                0.0f, 1.0f);

  /* 6. Profile contrast (0.14): post | dark gap | post */
  float profile    = gate_profile(lp, rp, h);

  /* 7. Isolation (0.08): soft version; hard veto already passed */
  float iso_score  = GATE_CLAMP((iso - GATE_ISOLATION_HARD_FLOOR)
                                / (1.0f - GATE_ISOLATION_HARD_FLOOR), 0.0f, 1.0f);

  /* 8. Checker bar (0.14): alternating B/W above posts */
  int16_t y_top_pair = GATE_MIN2(lp->y0, rp->y0);
  float checker    = gate_checker(buf, w, h, lp, rp, y_top_pair, pair_h);

  /* ── Weighted sum ─────────────────────────────────────────────────────── */
  float score =
      0.16f * h_ratio    +
      0.12f * overlap    +
      0.10f * gap_score  +
      0.10f * asp_score  +
      0.16f * purity     +
      0.14f * profile    +
      0.08f * iso_score  +
      0.14f * checker;

  /* Extra penalty for very low overlap (posts barely touch vertically) */
  if (overlap < 0.40f) score -= (0.40f - overlap) * 0.25f;

  return GATE_CLAMP(score, 0.0f, 1.0f);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 6 — Best pair selection  (Stage 4)
 * ═════════════════════════════════════════════════════════════════════════════*/

static bool gate_find_best_pair(const uint8_t *buf, uint16_t w, uint16_t h,
                                 const GPost *posts, uint8_t n_posts,
                                 struct gate_detection_t *out)
{
  float best_score = gate_min_confidence - 0.001f;
  bool  found = false;

  for (uint8_t i = 0; i < n_posts; i++) {
    for (uint8_t j = (uint8_t)(i + 1u); j < n_posts; j++) {
      float sc = gate_score_pair(buf, w, h, &posts[i], &posts[j]);
      if (sc < gate_min_confidence || sc <= best_score) continue;

      /* Determine which is left and which is right */
      const GPost *lp = &posts[i];
      const GPost *rp = &posts[j];
      if (lp->cx > rp->cx) { const GPost *t = lp; lp = rp; rp = t; }

      best_score = sc;
      found      = true;

      int16_t x_in1 = lp->x1 + 1;
      int16_t x_in2 = rp->x0 - 1;
      int16_t y_top = GATE_MIN2(lp->y0, rp->y0);
      int16_t y_bot = GATE_MAX2(lp->y1, rp->y1);

      out->valid          = true;
      out->confidence     = sc;
      out->opening_w      = (int16_t)(x_in2 - x_in1);
      out->opening_h      = (int16_t)(y_bot - y_top);
      out->center_x       = (int16_t)((x_in1 + x_in2) / 2);
      out->center_y       = (int16_t)((y_top + y_bot) / 2);
      out->image_w        = w;
      out->image_h        = h;
      out->lateral_error  = ((float)out->center_x - 0.5f * (float)w) / (0.5f * (float)w);
      out->vertical_error = ((float)out->center_y - 0.5f * (float)h) / (0.5f * (float)h);

      /* Yaw proxy from post-width asymmetry (matches Python yaw_proxy) */
      float dw = (float)(rp->w - lp->w) / (float)(rp->w + lp->w + 1);
      float dh = (float)(rp->h - lp->h) / (float)(rp->h + lp->h + 1);
      out->yaw_proxy = 0.65f * dw + 0.35f * dh;
    }
  }

  return found;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 7 — Optional image annotation  (debug aid)
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * gate_draw_rect — draw a rectangle on a UYVY image buffer.
 *
 * Sets pixels on the rectangle edge to the given Y, U, V values.
 * Green (Y=150, U=44, V=21) for valid; orange (Y=150, U=90, V=200) for sub-threshold.
 */
static void gate_draw_rect(uint8_t *buf, uint16_t w, uint16_t h,
                            int16_t x0, int16_t y0, int16_t bw, int16_t bh,
                            uint8_t Yv, uint8_t Uv, uint8_t Vv)
{
  if (x0 < 0 || y0 < 0 || bw < 2 || bh < 2) return;
  int16_t x1 = x0 + bw - 1, y1 = y0 + bh - 1;
  if (x1 >= (int16_t)w) x1 = (int16_t)w - 1;
  if (y1 >= (int16_t)h) y1 = (int16_t)h - 1;

  uint16_t stride = (uint16_t)(w * 2u);

  /* Draw top and bottom edges */
  for (int16_t x = x0; x <= x1; x++) {
    for (int16_t edge_y = y0; edge_y <= y1; edge_y += (bh - 1)) {
      uint8_t *p = buf + (uint32_t)edge_y * stride + ((uint16_t)x >> 1u) * 4u;
      p[0] = Uv;
      p[1 + ((uint16_t)x & 1u) * 2u] = Yv;
      p[2] = Vv;
      if (edge_y != y0 && edge_y != y1) break;
    }
  }
  /* Draw left and right edges */
  for (int16_t y = y0; y <= y1; y++) {
    for (int16_t edge_x = x0; edge_x <= x1; edge_x += (bw - 1)) {
      uint8_t *p = buf + (uint32_t)y * stride + ((uint16_t)edge_x >> 1u) * 4u;
      p[0] = Uv;
      p[1 + ((uint16_t)edge_x & 1u) * 2u] = Yv;
      p[2] = Vv;
      if (edge_x != x0 && edge_x != x1) break;
    }
  }
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 8 — Video callback  (runs in video thread)
 * ═════════════════════════════════════════════════════════════════════════════*/

static struct image_t *gate_detector_func(struct image_t *img, uint8_t __attribute__((unused)) id)
{
  if (img->type != IMAGE_YUV422) return img;

  uint16_t w = img->w;
  uint16_t h = img->h;
  uint8_t *buf = (uint8_t *)img->buf;

  if (w > GATE_COL_BUF) return img;   /* image wider than our buffers → skip */

  /* ── Stage 1: column scan ─────────────────────────────────────────────── */
  uint32_t total_blue = gate_scan_columns(buf, w, h);

  /* Global density gate: if more than 28 % of frame is blue, skip */
  struct gate_detection_t local;
  memset(&local, 0, sizeof(local));
  local.image_w = w;
  local.image_h = h;

  uint32_t total_px = (uint32_t)w * (uint32_t)h;
  if ((float)total_blue > GATE_MAX_MASK_DENSITY * (float)total_px) {
    goto publish;   /* frame flooded with blue → no detection */
  }

  /* ── Stage 2: find post candidates ───────────────────────────────────── */
  GPost posts[GATE_MAX_POSTS];
  uint8_t n_posts = gate_find_posts(posts, GATE_MAX_POSTS, w, h);

  if (n_posts < 2) goto publish;

  /* ── Stages 3 & 4: score pairs, pick best ─────────────────────────────── */
  gate_find_best_pair(buf, w, h, posts, n_posts, &local);

  /* ── Optional annotation on the image stream ──────────────────────────── */
  if (gate_debug_overlay && local.valid) {
    int16_t bx  = (int16_t)(local.center_x - local.opening_w / 2);
    int16_t by  = (int16_t)(local.center_y - local.opening_h / 2);
    /* Outer box: green */
    gate_draw_rect(buf, w, h, bx - 2, by - 2,
                   (int16_t)(local.opening_w + 4), (int16_t)(local.opening_h + 4),
                   150, 44, 21);
    /* Inner box: cyan */
    gate_draw_rect(buf, w, h, bx, by, local.opening_w, local.opening_h,
                   200, 180, 40);
  }

publish:
  pthread_mutex_lock(&_gate_mutex);
  memcpy(&_gate_shared, &local, sizeof(local));
  pthread_mutex_unlock(&_gate_mutex);

  return img;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SECTION 9 — Module init and periodic  (Paparazzi entry points)
 * ═════════════════════════════════════════════════════════════════════════════*/

/*
 * gate_detector_init — called once at autopilot start-up.
 *
 * Registers the video callback via cv_add_to_device.
 * Make sure `video_thread` is included before `gate_detector` in the airframe.
 */
void gate_detector_init(void)
{
  pthread_mutex_init(&_gate_mutex, NULL);
  memset(&_gate_shared, 0, sizeof(_gate_shared));
  memset(&gate_latest,  0, sizeof(gate_latest));

  _listener = cv_add_to_device(&GATE_DETECTOR_CAMERA,
                                gate_detector_func,
                                GATE_DETECTOR_FPS,
                                GATE_DETECTOR_ID);

  printf("[gate_detector] init: camera=%s  fps=%d  u_min=%d  v_max=%d  min_conf=%.2f\n",
         "GATE_DETECTOR_CAMERA", GATE_DETECTOR_FPS,
         (int)gate_u_min, (int)gate_v_max, (double)gate_min_confidence);
}

/*
 * gate_detector_periodic — called from the autopilot loop at 15 Hz.
 *
 * 1. Thread-safely copies the latest detection into gate_latest.
 * 2. Publishes via VISUAL_DETECTION ABI message so other modules can subscribe.
 *
 * Navigation example (GUIDED mode — add to your own module's periodic):
 * ─────────────────────────────────────────────────────────────────────
 *   #include "modules/gate_detector/gate_detector.h"
 *   #include "firmwares/rotorcraft/guidance/guidance_h.h"
 *
 *   void my_nav_periodic(void) {
 *       if (gate_latest.valid) {
 *           // gate_latest.lateral_error  ∈ [-1,+1], 0 = centred horizontally
 *           // gate_latest.vertical_error ∈ [-1,+1], 0 = centred vertically
 *           float vx = -gate_latest.lateral_error  * 0.3f;  // forward speed
 *           float vy = -gate_latest.vertical_error * 0.2f;  // sideways speed
 *           guidance_h_set_guided_body_vel(vx, vy);
 *       } else {
 *           guidance_h_set_guided_body_vel(0, 0);  // hover
 *       }
 *   }
 */
void gate_detector_periodic(void)
{
  /* Copy latest detection from video thread */
  pthread_mutex_lock(&_gate_mutex);
  memcpy(&gate_latest, &_gate_shared, sizeof(gate_latest));
  pthread_mutex_unlock(&_gate_mutex);

  /*
   * Publish via VISUAL_DETECTION ABI message.
   * Subscribers (e.g. a gate-follower nav module) receive:
   *   pixel_x     = opening centre x (image pixels)
   *   pixel_y     = opening centre y (image pixels)
   *   pixel_width = opening width    (image pixels)
   *   pixel_height= opening height   (image pixels)
   *   quality     = confidence × 10000  (int32; divide by 10000 to get float)
   *   extra       = 1 if valid, 0 if not
   */
  AbiSendMsgVISUAL_DETECTION(
      GATE_DETECTOR_ID,
      (int16_t)gate_latest.center_x,
      (int16_t)gate_latest.center_y,
      (int16_t)gate_latest.opening_w,
      (int16_t)gate_latest.opening_h,
      (int32_t)(gate_latest.confidence * 10000.0f),
      (int16_t)(gate_latest.valid ? 1 : 0)
  );

#ifdef GATE_DETECTOR_VERBOSE
  if (gate_latest.valid) {
    printf("[gate_detector] conf=%.2f  ex=%+.2f  ey=%+.2f  yaw=%+.2f  "
           "cx=%d cy=%d  ow=%d oh=%d\n",
           (double)gate_latest.confidence,
           (double)gate_latest.lateral_error,
           (double)gate_latest.vertical_error,
           (double)gate_latest.yaw_proxy,
           gate_latest.center_x, gate_latest.center_y,
           gate_latest.opening_w, gate_latest.opening_h);
  }
#endif
}
