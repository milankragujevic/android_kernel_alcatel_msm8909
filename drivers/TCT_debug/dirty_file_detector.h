#include <linux/mount.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
#include "../../../fs/mount.h"
#endif
#ifdef CONFIG_TCT_DIRTY_FILES_DETECTOR
extern int mark_system_dirty(const char *file_name);
extern int is_system_dirty(void);
#else
static inline int mark_system_dirty(const char *file_name) {return 0;};
static inline int is_system_dirty(void) {return 0;};
#endif
