#ifdef __cplusplus
extern "C" {
#endif

#ifndef INTERFACE_VEL_H
#define INTERFACE_VEL_H

#define CONCATENATE_INPUT(x) vel_##x

/* Function definitions in testsolver.c: */
int CONCATENATE_INPUT(initialize_optimizer)(void);
int CONCATENATE_INPUT(optimize)(void);
void CONCATENATE_INPUT(load_default_data)(double dt, double v_max, double x0[], double xf[], double q);
double CONCATENATE_INPUT(get_cost)(void);
double **CONCATENATE_INPUT(get_state)(void);
double **CONCATENATE_INPUT(get_control)(void);

#undef CONCATENATE_INPUT

#endif
#ifdef __cplusplus
}
#endif
