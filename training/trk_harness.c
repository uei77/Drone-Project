/* Runs the firmware tracker.c on probability streams.
 * stdin : int32 n_streams, int32 n_frames, int32 K, then float32 probs[n][L][K], uint32 t_ms[n][L]
 * argv  : thr thr_low n_on m_off cooldown_ms
 * stdout: "stream frame kind sit" per event */
#include <stdio.h>
#include <stdlib.h>
#include "tracker.h"

int main(int argc, char **argv)
{
    int n, L, K;
    if (fread(&n, 4, 1, stdin) != 1 || fread(&L, 4, 1, stdin) != 1 || fread(&K, 4, 1, stdin) != 1) return 1;
    float *p = malloc(sizeof(float) * n * L * K);
    unsigned *tm = malloc(sizeof(unsigned) * n * L);
    if (fread(p, sizeof(float), (size_t)n * L * K, stdin) != (size_t)n * L * K) return 2;
    if (fread(tm, 4, (size_t)n * L, stdin) != (size_t)n * L) return 3;
    trk_cfg_t cfg[TRK_MAX];
    for (int i = 0; i < K; i++) {
        trk_cfg_t c = { (float)atof(argv[1]), (float)atof(argv[2]), (uint8_t)atoi(argv[3]),
                        (uint8_t)atoi(argv[4]), 1, (uint32_t)atoi(argv[5]), 30000 };
        cfg[i] = c;
    }
    for (int s = 0; s < n; s++) {
        tracker_t t;
        tracker_init(&t, cfg, K);
        for (int f = 0; f < L; f++) {
            trk_ev_t e = tracker_update(&t, p + ((size_t)s * L + f) * K, 1, tm[s * L + f]);
            if (e.kind != TRK_EV_NONE) printf("%d %d %d %d\n", s, f, e.kind, e.sit);
        }
    }
    return 0;
}
