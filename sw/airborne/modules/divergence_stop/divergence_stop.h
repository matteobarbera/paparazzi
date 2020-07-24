#ifndef DIVERGENCE_STOP_H
#define DIVERGENCE_STOP_H

extern float divergence_stop_threshold;

extern void divergence_stop_init(void);
extern void divergence_stop_periodic(void);

extern void divergence_stop_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);

bool is_divergence_stop_active(void);

#endif
