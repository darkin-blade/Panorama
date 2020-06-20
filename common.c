#include <common.h>

static char sprint_buf[1024];
static char tmp_fmt[1024];

void fuck(const char *fmt, const char *func, const int line_no, ...) {
  va_list args;
  int n;
  va_start(args, fmt);

#if defined(UBUNTU)
  sprintf(tmp_fmt, "\033[1;32m%s\33[0m\n", fmt);
#else
  sprintf(tmp_fmt, "%s", fmt);
#endif

  n = vsprintf(sprint_buf, tmp_fmt, args);
  va_end(args);
  write(STDOUT_FILENO, sprint_buf, n);
}
