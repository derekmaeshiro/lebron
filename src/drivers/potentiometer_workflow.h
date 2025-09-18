// potentiometer_workflow.h

#ifndef POTENTIOMETER_WORKFLOW_H
#define POTENTIOMETER_WORKFLOW_H

#include <stdbool.h>

extern bool workflow_enabled;

// Function declarations, no static variables!
void potentiometer_workflow_init(void);
void potentiometer_workflow_enable(void);
void potentiometer_workflow_disable(void);
void potentiometer_workflow_run(void);

#endif