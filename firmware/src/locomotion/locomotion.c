/* Forward pass for the generated weights; layers are row-major [out][in]. */
#include "locomotion.h"
#include "generated_locomotion_weights.h"
#include <math.h>

#if BIPE_LOCOMOTION_OBS != BIPE_LOCOMOTION_OBSERVATIONS || BIPE_LOCOMOTION_ACT != BIPE_LOCOMOTION_ACTIONS
#error "generated_locomotion_weights.h does not match locomotion.h"
#endif

static void dense(const float *w, const float *b, const float *x, float *y, int rows, int cols, int elu) {
    for (int i = 0; i < rows; i++) {
        const float *row = w + i * cols;
        float s = b[i];
        for (int j = 0; j < cols; j++) s += row[j] * x[j];
        y[i] = (elu && s < 0.0f) ? expm1f(s) : s;
    }
}

void bipe_locomotion_forward(const float observation[BIPE_LOCOMOTION_OBSERVATIONS],
                             float action[BIPE_LOCOMOTION_ACTIONS]) {
    float x[BIPE_LOCOMOTION_OBS], h1[BIPE_LOCOMOTION_H1], h2[BIPE_LOCOMOTION_H2], h3[BIPE_LOCOMOTION_H3];
    for (int i = 0; i < BIPE_LOCOMOTION_OBS; i++)
        x[i] = (observation[i] - bipe_locomotion_mean[i]) * bipe_locomotion_scale[i];
    dense(bipe_locomotion_w0, bipe_locomotion_b0, x, h1, BIPE_LOCOMOTION_H1, BIPE_LOCOMOTION_OBS, 1);
    dense(bipe_locomotion_w1, bipe_locomotion_b1, h1, h2, BIPE_LOCOMOTION_H2, BIPE_LOCOMOTION_H1, 1);
    dense(bipe_locomotion_w2, bipe_locomotion_b2, h2, h3, BIPE_LOCOMOTION_H3, BIPE_LOCOMOTION_H2, 1);
    dense(bipe_locomotion_w3, bipe_locomotion_b3, h3, action, BIPE_LOCOMOTION_ACT, BIPE_LOCOMOTION_H3, 0);
}

const char *bipe_locomotion_checkpoint(void) { return BIPE_LOCOMOTION_CHECKPOINT_SHA256; }
