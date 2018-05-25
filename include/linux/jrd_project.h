#ifndef __JRD_PROJECT_H
#define __JRD_PROJECT_H
#ifdef  TCT_SW_ALL_IN_ONE
#include <linux/string.h>
extern char *static_version_trace;
#define ISVERSION(version_trace)  (!strcmp(static_version_trace,version_trace))
#define PROJECT_TRACE()  (static_version_trace)
#endif
#endif