#include "potentiometer.h"
#include <stdbool.h>

#ifndef POTENTIOMETER_WORKFLOW_H
#define POTENTIOMETER_WORKFLOW_H
extern bool workflow_enabled;
;
void potentiometer_workflow_init(void);
void potentiometer_workflow_enable(void);
void potentiometer_workflow_disable(void);
// The following method is to be used in a never-ending loop
void potentiometer_workflow_run(void);
#endif