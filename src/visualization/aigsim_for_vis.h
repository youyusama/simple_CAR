#ifndef aigsim_for_vis_h_INCLUDE
#define aigsim_for_vis_h_INCLUDE

#include "../model/aiger.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <ctype.h>

static FILE *file;
static int close_file;
static unsigned char *current;
static unsigned char *next;
static aiger *model;
static int filter;

static void
die (const char *fmt, ...)
{
  va_list ap;
  fputs ("*** [aigsim] ", stderr);
  va_start (ap, fmt);
  vfprintf (stderr, fmt, ap);
  va_end (ap);
  fputc ('\n', stderr);
  exit (1);
}

static unsigned char
deref (unsigned lit)
{
  unsigned res = current[aiger_lit2var (lit)];
  res ^= aiger_sign (lit);
#ifndef NDEBUG
  if (lit == 0)
    assert (res == 0);
  if (lit == 1)
    assert (res == 1);
#endif
  return res;
}

static void
put (unsigned lit)
{
  unsigned v = deref (lit);
  if (v & 2)
    fputc ('x', stdout);
  else
    fputc ('0' + (v & 1), stdout);
}

static const char *
idx_as_vcd_id (char ch, unsigned idx)
{
  static char buffer[20];
  sprintf (buffer, "%c%u", ch, idx);
  return buffer;
}

static const char *
aiger_symbol_as_string (aiger_symbol * s)
{
  static char buffer[20];
  if (s->name)
    return s->name;

  sprintf (buffer, "%u", s->lit / 2);
  return buffer;
}

static void
print_vcd_symbol (const char *symbol)
{
  const char *p;
  char ch;

  for (p = symbol; (ch = *p); p++)
    fputc ((isspace (ch) ? '"' : ch), stdout);
}

static int last = EOF;
static int first = 0;

static int ignore_line_starting_with (int ch)
{
  if (ch == 'c') return 1;
  if (ch == 'u') return 1;
  if (!filter) return 0;
  if (last != '\n') return 0;
  if (ch == '0') return 0;
  if (ch == '1') return 0;
  if (ch == 'b') return 0;
  if (ch == 'x') return 0;
  if (ch == '.') return 0;
  return 1;
}

static int
nxtc (FILE * file)
{
  int start, ch;
RESTART:
  start = getc (file);
  if (start == EOF) return start;
  if (filter && last == EOF) {
    if (start != '0' && start != '1')
      {
IGNORE_REST_OF_LINE:
	assert (last == '\n' || last == EOF);
	while ((ch = getc (file)) != '\n')
	  if (ch == EOF)
	    die ("unexpected EOF after '%c'", start);
	goto RESTART;
      }
    ch = getc (file);
    if (ch != '\n')
      goto IGNORE_REST_OF_LINE;
    ungetc (ch, file);
    return last = start;
  }
  if (ignore_line_starting_with (start))
    goto IGNORE_REST_OF_LINE;
  last = start;
  return start;
}


#define ALLOC_STATES 100

void aigsim_for_vis (u_int32_t** a, const char* aiger_file_name, const char* cex_file_name, int step_num);

#endif