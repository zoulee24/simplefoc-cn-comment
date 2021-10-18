#ifndef __PRINT_H
#define __PRINT_H

class Print{
	public:
	void print(char * msg);
	void print(float msg);
	void print(float msg, int unknown);
	void print(int msg);
	void print(double msg);
	void println(char * msg);
	void println(void);
};

#endif // __PRINT_H