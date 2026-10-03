#include "situations.h"

/*
 * Situation table for the CURRENT model (synthetic demo classes), written by hand.
 * training/export_c.py replaces this file together with model_data.c / model_meta.h when a
 * model trained on the real situation list (training/situations.json) is exported; from then
 * on thresholds and responses are changed in situations.json, not here.
 *
 *  name        thr   low  n_on m_off prio cooldown  max_active   response            urg hold
 */
const sit_def_t SITUATIONS[MODEL_NUM_CLASSES] = {
    { "normal",   { 1.00f, 1.00f, 0, 0, 0,     0,      0 }, RESP_NONE,        0,     0 },
    { "approach", { 0.80f, 0.60f, 4, 4, 3, 10000,  15000 }, RESP_CLIMB_HOLD,  2,  2000 },
    { "recede",   { 0.80f, 0.60f, 4, 4, 1, 30000,  10000 }, RESP_REPORT,      0,     0 },
    { "static",   { 0.80f, 0.60f, 5, 4, 2, 60000,  15000 }, RESP_HOLD_REPORT, 1,  5000 },
    { "marker",   { 0.85f, 0.65f, 5, 6, 2, 30000,  30000 }, RESP_CENTER_LAND, 1,     0 },
};
/* app_main() checks at boot that every name matches MODEL_CLASS_NAMES in order. */
