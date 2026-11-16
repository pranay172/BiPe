/* Bench parity check of the forward pass; see learned.h. */
#include "learned.h"
#include "generated_locomotion_parity.h"
#include <math.h>
#include <string.h>

bipe_parity_result bipe_locomotion_parity(bipe_forward_fn forward, uint32_t (*cycles)(void)) {
    bipe_parity_result r = {0, 0.0, UINT32_MAX, 0, false};
    if (!forward) return r;
    bool finite = true;
    for (unsigned k = 0; k < BIPE_LOCOMOTION_PARITY_VECTORS; k++) {
        float action[BIPE_LOCOMOTION_ACTIONS];
        uint32_t start = cycles ? cycles() : 0;
        forward(bipe_locomotion_parity_observation[k], action);
        uint32_t spent = cycles ? cycles() - start : 0;
        if (spent < r.min_cycles) r.min_cycles = spent;
        if (spent > r.max_cycles) r.max_cycles = spent;
        for (unsigned i = 0; i < BIPE_LOCOMOTION_ACTIONS; i++) {
            double error = fabs((double)action[i] - bipe_locomotion_parity_action[k][i]);
            finite = finite && isfinite(error);
            if (error > r.max_error) r.max_error = error;
        }
        r.vectors++;
    }
    /* The vectors must come from the checkpoint the linked weights came from. */
    r.passed = finite && r.max_error <= BIPE_LOCOMOTION_PARITY_TOLERANCE &&
               strcmp(bipe_locomotion_checkpoint(), BIPE_LOCOMOTION_PARITY_CHECKPOINT) == 0 &&
               r.vectors == BIPE_LOCOMOTION_PARITY_VECTORS;
    return r;
}
