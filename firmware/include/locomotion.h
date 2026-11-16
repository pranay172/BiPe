/* M13 learned locomotion: the settle policy's forward pass (bipe/m13/export.py).
   Float32, no allocation, no state: the caller owns the observation and the held
   action, and the C supervisor stays in charge of the gate and the protective fall. */
#ifndef BIPE_LOCOMOTION_H
#define BIPE_LOCOMOTION_H

#define BIPE_LOCOMOTION_OBSERVATIONS 25
#define BIPE_LOCOMOTION_ACTIONS 6

/* action = MLP((observation - mean) * scale): Linear-ELU 256, 128, 64, Linear 6. */
void bipe_locomotion_forward(const float observation[BIPE_LOCOMOTION_OBSERVATIONS],
                             float action[BIPE_LOCOMOTION_ACTIONS]);
/* sha256 of the checkpoint the compiled weights came from. */
const char *bipe_locomotion_checkpoint(void);

#endif
