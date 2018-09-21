#include "KBAsync.h"

struct termios orig_termios;
void reset_terminal_mode() 
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

KBAsync::KBAsync()
{
	 struct termios new_termios;

	 /* take two copies - one for now, one for later */
	 tcgetattr(0, &orig_termios);
	 memcpy(&new_termios, &orig_termios, sizeof(new_termios));

	 /* register cleanup handler, and set the new terminal mode */
	 atexit(reset_terminal_mode);
	 cfmakeraw(&new_termios);
	 tcsetattr(0, TCSANOW, &new_termios);
}

KBAsync::~KBAsync()
{
	 reset_terminal_mode();
}

int KBAsync::kbhit()
{
	 struct timeval tv = { 0L, 0L };
	 fd_set fds;
	 FD_ZERO(&fds);
	 FD_SET(0, &fds);
	 return select(1, &fds, NULL, NULL, &tv);
}

int KBAsync::getch()
{
	int r;
	unsigned char c;
	
	if ((r = read(0, &c, sizeof(c))) < 0)
		return r;
	else
		return c;
}

int KBAsync::getKey()
{
	if (kbhit()) return getch();
		return -1;
}

