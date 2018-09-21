#ifndef KBAsync_H
#define KBAsync_H

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

class KBAsync
{
	public:
		KBAsync();
		~KBAsync();
		int kbhit();
		int getch();
		int getKey();
};

#endif
